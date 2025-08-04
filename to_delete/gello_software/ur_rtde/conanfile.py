from conans import ConanFile
from conans.tools import os_info

class UR_Rtde(ConanFile):
    settings = 'os', 'compiler', 'build_type', 'arch'
    generators = 'cmake'

    def requirements(self):
        self.requires('boost/1.75.0')

    def imports(self):
        self.copy('*.dll', dst='bin', src='bin')
