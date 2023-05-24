# Copyright 2023 Magazino GmbH
# Copyright 2022 Intelligent Robotics Lab
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

from os.path import exists
import rospkg


def get_domain_and_problem(path_domain, path_problem):
    rospack = rospkg.RosPack()
    # needed to make the github pipelines work
    domain = rospack.get_path("up4ros") + "/tests/" + path_domain
    problem = rospack.get_path("up4ros") + "/tests/" + path_problem

    if not exists(domain):
        domain = rospack.get_path("up4ros") + "/" + path_domain
        problem = rospack.get_path("up4ros") + "/" + path_problem

    return domain, problem
