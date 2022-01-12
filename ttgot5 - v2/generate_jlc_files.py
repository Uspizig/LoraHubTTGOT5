# Copyright (C) 2019 Matthew Lai
#
# This file is part of JLC Kicad Tools.
#
# JLC Kicad Tools is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# JLC Kicad Tools is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with JLC Kicad Tools.  If not, see <https://www.gnu.org/licenses/>.

import os
import re
import sys

from jlc_lib.cpl_fix_rotations import ReadDB, FixRotations
from jlc_lib.generate_bom import GenerateBOM

DB_PATH="cpl_rotations_db.csv"

def main():
	if len(sys.argv) != 2:
		print("Usage: {} <project directory>".format(sys.argv[0]))
		return
	project_dir = sys.argv[1]
	if not os.path.isdir(project_dir):
		print("Failed to open project directory: {}".format(project_dir))
		return

	project_name = os.path.basename(os.path.normpath(project_dir))
	netlist_filename = project_name + ".xml"
	cpl_filename = project_name + "-all-pos.csv"
	netlist_path = None
	cpl_path = None

	for dir_name, subdir_list, file_list in os.walk(project_dir):
		for file_name in file_list:
			if file_name == netlist_filename:
				netlist_path = os.path.join(dir_name, file_name)
			elif file_name == cpl_filename:
				cpl_path = os.path.join(dir_name, file_name)

	if netlist_path is None:
		print((
			"Failed to find netlist file: {} in {} (and sub-directories). "
			"Run 'Tools -> Generate Bill of Materials' in Eeschema (any format). "
			"It will generate an intermediate file we need.").format(netlist_filename, project_dir))
		return

	if cpl_path is None:
		print((
			"Failed to find CPL file: {} in {} (and sub-directories). "
			"Run 'File -> Fabrication Outputs -> Footprint Position (.pos) File' in Pcbnew. "
			"Settings: 'CSV', 'mm', 'single file for board'.").format(cpl_filename, project_dir))
		return

	print("Netlist file found at: {}".format(netlist_path))
	print("CPL file found at: {}".format(cpl_path))

	bom_output_path = os.path.join(project_dir, project_name + "_bom_jlc.csv")
	cpl_output_path = os.path.join(project_dir, project_name + "_cpl_jlc.csv")

	db = ReadDB(DB_PATH)
	if GenerateBOM(netlist_path, bom_output_path) and FixRotations(cpl_path, cpl_output_path, db):
		print("")
		print("JLC BOM file written to: {}".format(bom_output_path))
		print("JLC CPL file written to: {}".format(cpl_output_path))


if __name__ == '__main__':
  main()