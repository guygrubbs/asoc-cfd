# Created by Microsemi Libero Software 11.9.6.7
# Tue May 05 18:12:33 2026

# (OPEN DESIGN)

open_design "fullsystem_top.adb"

# set default back-annotation base-name
set_defvar "BA_NAME" "fullsystem_top_ba"
set_defvar "IDE_DESIGNERVIEW_NAME" {Impl1}
set_defvar "IDE_DESIGNERVIEW_COUNT" "1"
set_defvar "IDE_DESIGNERVIEW_REV0" {Impl1}
set_defvar "IDE_DESIGNERVIEW_REVNUM0" "1"
set_defvar "IDE_DESIGNERVIEW_ROOTDIR" {C:\Users\aamar\Documents\SeniorSem2\4806-MDE\asoc-cfd\asoc-cfd\fpga\LiberoProject\FullSystem\designer}
set_defvar "IDE_DESIGNERVIEW_LASTREV" "1"


# import of input files
import_source  \
-format "edif" -edif_flavor "GENERIC" -netlist_naming "VERILOG" {../../synthesis/fullsystem_top.edn} -merge_physical "yes" -merge_timing "yes"
compile
report -type "status" {fullsystem_top_compile_report.txt}
report -type "pin" -listby "name" {fullsystem_top_report_pin_byname.txt}
report -type "pin" -listby "number" {fullsystem_top_report_pin_bynumber.txt}

save_design
