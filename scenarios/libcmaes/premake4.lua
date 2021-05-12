
project "libcmaes"
	language "C++"
	-- kind "SharedLib"
	kind "StaticLib"

	includedirs {
		"src/",
		-- "Eigen/",
		"../",
		"./"
	}

	files {
		"src/*.h",
		"src/*.cpp",
		"src/*.cc",
	}
	
	configuration { "windows" }
		-- libdirs { "lib" }
		buildoptions { "/bigobj" }
		linkoptions { "/bigobj" }
		
	configuration { "macosx" }
		linkoptions { 
			"-install_name @rpath/liblibcmaes.dylib"
		}
		buildoptions("-std=c++0x -ggdb -fPIC" )	
		
	configuration { "linux" }
		buildoptions("-std=c++0x -ggdb -fPIC" )	


project "libcmaes-example"
	language "C++"
	kind "ConsoleApp"
	-- kind "StaticLib"


	includedirs {
		"src/",
		"../",
		"./",
	}

	links {
		"libcmaes",
	}

	files {
		"sample-code.cc",
	}

	targetdir "../../build/bin"
	
	configuration { "windows" }
		-- libdirs { "lib" }
		-- buildoptions { "/bigobj" }
		-- linkoptions { "/bigobj" }

	configuration { "macosx" }
		buildoptions("-std=c++0x -ggdb" )	
		
	configuration { "linux" }
		buildoptions("-std=c++0x -ggdb" )

--]====]
