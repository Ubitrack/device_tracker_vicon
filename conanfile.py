from conans import ConanFile, CMake, tools


class UbitrackCoreConan(ConanFile):
    name = "ubitrack_device_tracker_vicon"
    version = "1.3.0"

    description = "Ubitrack Device Tracker vicon"
    url = "https://github.com/Ubitrack/device_tracker_vicon.git"
    license = "GPL"

    short_paths = True
    settings = "os", "compiler", "build_type", "arch"
    generators = "cmake"

    options = {
        "workspaceBuild" : [True, False],
            }

    default_options = {
        "ubitrack_core:shared" : True,
        "ubitrack_dataflow:shared" : True,
        "workspaceBuild" : False,
        }

    def requirements(self):
        userChannel = "ubitrack/stable"
        if self.options.workspaceBuild:
            userChannel = "local/dev"


        self.requires("ubitrack_core/%s@%s" % (self.version, userChannel))
        self.requires("ubitrack_dataflow/%s@%s" % (self.version, userChannel) )
        self.requires("vicon-datastream-sdk/1.8.0@vendor/stable")

    # all sources are deployed with the package
    exports_sources = "doc/*", "src/*", "CMakeLists.txt"

    # def imports(self):
    #     self.copy(pattern="*.dll", dst="bin", src="bin") # From bin to bin
    #     self.copy(pattern="*.dylib*", dst="lib", src="lib") 
    #     self.copy(pattern="*.so*", dst="lib", src="lib") 
       
    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()
        cmake.install()

    def package(self):
        pass

    def package_info(self):
        pass
