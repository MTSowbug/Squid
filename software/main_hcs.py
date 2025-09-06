# set QT_API environment variable
import argparse
import glob
import logging
import os
import csv

os.environ["QT_API"] = "pyqt5"
import signal
import sys

# qt libraries
from qtpy.QtWidgets import *
from qtpy.QtGui import *

import squid.logging

squid.logging.setup_uncaught_exception_logging()

# app specific libraries
import control.gui_hcs as gui
from configparser import ConfigParser
from control.widgets import ConfigEditorBackwardsCompatible, StageUtils
from control._def import CACHED_CONFIG_FILE_PATH
from control._def import USE_TERMINAL_CONSOLE
import control.utils
import control.microscope
import control.core.core as core
from control.microscope import ScanCoordinatesSiLA2


if USE_TERMINAL_CONSOLE:
    from control.console import ConsoleThread


def show_config(cfp, configpath, main_gui):
    config_widget = ConfigEditorBackwardsCompatible(cfp, configpath, main_gui)
    config_widget.exec_()


def load_scan_coordinates(csv_path, microscope):
    """Load scan coordinates from a CSV file.

    The CSV should contain at least columns named ``region``, ``x (mm)``,
    ``y (mm)``, and optionally ``z (mm)``.  The file format mirrors the
    coordinates.csv that is produced after an acquisition.
    """
    scan = ScanCoordinatesSiLA2(
        microscope.objective_store, microscope.camera.get_pixel_size_unbinned_um()
    )
    with open(csv_path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            region = row.get("region") or "region0"
            x = float(row.get("x (mm)") or row.get("x_mm") or row.get("x"))
            y = float(row.get("y (mm)") or row.get("y_mm") or row.get("y"))
            z_str = row.get("z (mm)") or row.get("z_mm") or row.get("z")
            if z_str is not None and z_str != "":
                coord = (x, y, float(z_str))
            else:
                coord = (x, y)

            if region not in scan.region_fov_coordinates:
                scan.region_fov_coordinates[region] = []
                scan.region_centers[region] = list(coord)

            scan.region_fov_coordinates[region].append(coord)

    return scan


def run_headless_acquisition(args, log):
    """Run a multipoint acquisition without launching the GUI."""

    microscope = control.microscope.Microscope.build_from_global_config(args.simulation)

    autofocus = core.AutoFocusController(
        microscope.camera,
        microscope.stage,
        microscope.live_controller,
        microscope.low_level_drivers.microcontroller,
        microscope.addons.nl5,
    )

    if args.coordinates:
        scan_coordinates = load_scan_coordinates(args.coordinates, microscope)
    else:
        # Acquire current field of view
        pos = microscope.stage.get_pos()
        scan_coordinates = ScanCoordinatesSiLA2(
            microscope.objective_store, microscope.camera.get_pixel_size_unbinned_um()
        )
        scan_coordinates.region_centers["current"] = [pos.x_mm, pos.y_mm, pos.z_mm]
        scan_coordinates.region_fov_coordinates["current"] = [
            (pos.x_mm, pos.y_mm, pos.z_mm)
        ]

    mp = core.MultiPointController(
        microscope.camera,
        microscope.stage,
        microscope.addons.piezo_stage,
        microscope.low_level_drivers.microcontroller,
        microscope.live_controller,
        autofocus,
        microscope.objective_store,
        microscope.channel_configuration_manager,
        scan_coordinates=scan_coordinates,
        fluidics=microscope.addons.fluidics,
        headless=True,
    )

    if args.base_path is None or args.experiment_id is None:
        log.error("base-path and experiment-id are required for headless acquisition")
        sys.exit(1)

    mp.set_base_path(args.base_path)
    mp.start_new_experiment(args.experiment_id)

    if args.configurations:
        configs = args.configurations.split(",")
    else:
        configs = [c.name for c in microscope.channel_configuration_manager.configurations[:1]]
    mp.set_selected_configurations(configs)
    mp.set_deltaZ(args.deltaZ)
    mp.set_NZ(args.NZ)
    mp.set_deltat(args.deltat)
    mp.set_Nt(args.Nt)
    mp.set_use_piezo(args.use_piezo)
    mp.set_af_flag(args.autofocus)
    mp.set_reflection_af_flag(args.reflection_af)

    mp.run_acquisition(acquire_current_fov=not bool(args.coordinates))


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--simulation", help="Run the GUI with simulated hardware.", action="store_true")
    parser.add_argument("--live-only", help="Run the GUI only the live viewer.", action="store_true")
    parser.add_argument("--verbose", help="Turn on verbose logging (DEBUG level)", action="store_true")
    parser.add_argument(
        "--run-acquisition",
        help="Run a multipoint acquisition from the command line without launching the GUI",
        action="store_true",
    )
    parser.add_argument("--coordinates", help="CSV file with scan coordinates", default=None)
    parser.add_argument("--base-path", help="Directory to save acquired data", default=None)
    parser.add_argument("--experiment-id", help="Experiment identifier", default=None)
    parser.add_argument("--configurations", help="Comma separated list of imaging configurations", default=None)
    parser.add_argument("--deltaZ", type=float, default=0, help="Z step in microns")
    parser.add_argument("--NZ", type=int, default=1, help="Number of Z slices")
    parser.add_argument("--deltat", type=float, default=0, help="Time between time points in seconds")
    parser.add_argument("--Nt", type=int, default=1, help="Number of time points")
    parser.add_argument("--use-piezo", action="store_true", help="Use piezo for Z stacks")
    parser.add_argument("--autofocus", action="store_true", help="Enable contrast-based autofocus")
    parser.add_argument("--reflection-af", action="store_true", help="Enable reflection autofocus")
    args = parser.parse_args()

    log = squid.logging.get_logger("main_hcs")

    if args.verbose:
        log.info("Turning on debug logging.")
        squid.logging.set_stdout_log_level(logging.DEBUG)

    if not squid.logging.add_file_logging(f"{squid.logging.get_default_log_directory()}/main_hcs.log"):
        log.error("Couldn't setup logging to file!")
        sys.exit(1)

    log.info(f"Squid Repository State: {control.utils.get_squid_repo_state_description()}")

    if args.run_acquisition:
        run_headless_acquisition(args, log)
        sys.exit(0)

    legacy_config = False
    cf_editor_parser = ConfigParser()
    config_files = glob.glob("." + "/" + "configuration*.ini")
    if config_files:
        cf_editor_parser.read(CACHED_CONFIG_FILE_PATH)
    else:
        log.error("configuration*.ini file not found, defaulting to legacy configuration")
        legacy_config = True
    app = QApplication([])
    app.setStyle("Fusion")
    # This allows shutdown via ctrl+C even after the gui has popped up.
    signal.signal(signal.SIGINT, signal.SIG_DFL)

    microscope = control.microscope.Microscope.build_from_global_config(args.simulation)
    win = gui.HighContentScreeningGui(
        microscope=microscope, is_simulation=args.simulation, live_only_mode=args.live_only
    )

    file_menu = QMenu("File", win)

    if not legacy_config:
        config_action = QAction("Microscope Settings", win)
        config_action.triggered.connect(lambda: show_config(cf_editor_parser, config_files[0], win))
        file_menu.addAction(config_action)

    microscope_utils_menu = QMenu("Utils", win)

    stage_utils_action = QAction("Stage Utils", win)
    stage_utils_action.triggered.connect(win.stageUtils.show)
    microscope_utils_menu.addAction(stage_utils_action)

    try:
        csw = win.cswWindow
        if csw is not None:
            csw_action = QAction("Camera Settings", win)
            csw_action.triggered.connect(csw.show)
            file_menu.addAction(csw_action)
    except AttributeError:
        pass

    try:
        csw_fc = win.cswfcWindow
        if csw_fc is not None:
            csw_fc_action = QAction("Camera Settings (Focus Camera)", win)
            csw_fc_action.triggered.connect(csw_fc.show)
            file_menu.addAction(csw_fc_action)
    except AttributeError:
        pass

    menu_bar = win.menuBar()
    menu_bar.addMenu(file_menu)
    menu_bar.addMenu(microscope_utils_menu)
    win.show()

    if USE_TERMINAL_CONSOLE:
        console_locals = {"microscope": win.microscope}
        console_thread = ConsoleThread(console_locals)
        console_thread.start()

    sys.exit(app.exec_())
