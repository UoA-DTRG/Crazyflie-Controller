import vicon_core_api
from vicon_core_api import *

import tracker_api
from tracker_api import BasicObjectServices

c = Client('localhost')
print(c.connected)

services = BasicObjectServices(c)
result, object_list = services.basic_object_list()
print(result)
print(object_list)