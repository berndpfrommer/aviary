#! /usr/bin/env python3
# -----------------------------------------------------------------------------
# Copyright 2024 Bernd Pfrommer <bernd.pfrommer@gmail.com>
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

import argparse
import re
import subprocess


def modify_bag(input_file):
    in_section = False
    version_pattern = re.compile('  version: ')
    version_pattern_5 = re.compile('  version: 5')
    start_pattern = re.compile('.*offered_qos_profiles:.*')
    end_pattern = re.compile('.*avoid_ros_namespace_conventions: false')
    backup_file = input_file + '/metadata.yaml.orig'
    modified_file = input_file + '/metadata.yaml.mod'
    input_file += '/metadata.yaml'
    with open(input_file) as in_file:
        with open(modified_file, 'w') as mod_file:
            for line in in_file:
                if version_pattern.match(line):
                    if version_pattern_5.match(line):
                        raise Exception('bag is already version 5!')
                    mod_file.write(re.sub(r'version\:.*', 'version: 5', line))
                    print('original bag version: ', line.rstrip(), ' converted to 5')
                elif start_pattern.match(line):
                    mod_file.write(line.rstrip() + ' ""\n')
                    in_section = True
                else:
                    if not in_section:
                        mod_file.write(line)
                if end_pattern.match(line):
                    in_section = False
    output = subprocess.Popen(['cp', input_file, backup_file])
    output = subprocess.Popen(['cp', modified_file, input_file])
    output.communicate()  # Will block until launch complete done


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='modify bag metadata to version 5.')
    parser.add_argument('--bag', '-b', help='name of bag directory', default=None, required=True)

    args = parser.parse_args()

    modify_bag(args.bag)
