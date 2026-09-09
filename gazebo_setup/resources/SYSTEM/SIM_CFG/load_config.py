#!/usr/bin/env python

##
## Copyright (c) 2024 Numurus <https://www.numurus.com>.
##
## This file is part of nepi setup tools (nepi_setup) repo
## (see https://github.com/nepi-engine/nepi_setup)
##
## License: nepi setup tools are licensed under the "Numurus Software License",
## which can be found at: <https://numurus.com/wp-content/uploads/Numurus-Software-License-Terms.pdf>
##
## Redistributions in source code must retain this top-level comment block.
## Plagiarizing this software to sidestep the license obligations is illegal.
##
## Contact Information:
## ====================
## - mailto:nepi@numurus.com
##


# Generic YAML config loader: reads key/values from CONFIG_FILE (falling
# back to CONFIG_FILE + ".bak" for any keys missing after a failed or
# partial load) and prints them as KEY=VALUE pairs for a calling shell
# to eval/export.
#
# success codes (last "success=N" entry wins when the caller exports
# every entry in order):
#   1 = CONFIG_FILE itself parsed cleanly
#   2 = CONFIG_FILE was missing/invalid but the backup fully covered it
#   0 = nothing usable could be loaded
#
# Usage: load_config.py <config_file>


import os
import sys
import yaml


print_list=[]
def read_yaml_2_dict(file_path):
    if not os.path.exists(file_path):
        print_list.append("success=-2")
        return None
    try:
        with open(file_path) as f:
            dict_from_file = yaml.load(f, Loader=yaml.FullLoader)
    except Exception as e:
        dict_from_file = None
    if dict_from_file is None:
       print_list.append("success=-1")
    return dict_from_file

if len(sys.argv) < 2:
    print_list.append("success=0")
else:
    config_file = sys.argv[1]
    backup_file = config_file + ".bak"

    if os.path.exists(config_file) == True:
        config_dict = read_yaml_2_dict(config_file)
        primary_ok = config_dict is not None
        if config_dict is None:
            config_dict = dict()

        if os.path.exists(backup_file) == True:
            backup_dict = read_yaml_2_dict(backup_file)
            if backup_dict is not None:
                for key in backup_dict.keys():
                    if key not in config_dict.keys():
                        config_dict[key] = backup_dict[key]

        if config_dict:
            for key in config_dict.keys():
                print_string=(str(key) + "=" + str(config_dict[key]))
                print_list.append(print_string)
            print_list.append("success=1" if primary_ok else "success=2")
        else:
            print_list.append("success=0")
    else:
        print_list.append("success=0")

print_string="\'"
for entry in print_list:
    print_string += entry + " "
print_string += "\'"
print(print_string)
