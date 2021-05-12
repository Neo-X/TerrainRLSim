
solution "0_steersuite"
	configurations { 
		"Debug",
		"Release"
	}
	
	platforms {
		"x32", 
		"x64"
	}
	location (todir)

	-- extra warnings, no exceptions or rtti
	flags { 
		"ExtraWarnings",
--		"FloatFast",
--		"NoExceptions",
--		"NoRTTI",
		"Symbols"
	}
	defines { "ENABLE_GUI", "ENABLE_GLFW" }

	-- debug configs
	configuration "Debug*"
		defines { "DEBUG" }
		flags {
			"Symbols",
			Optimize = Off
		}
 
 	-- release configs
	configuration "Release*"
		defines { "NDEBUG" }
		flags { "Optimize" }

	-- windows specific
	configuration "windows"
		defines { "WIN32", "_WINDOWS" }
		libdirs { "lib" }
		targetdir ( "bin" )

	configuration { "linux" }
		linkoptions { 
			-- "-stdlib=libc++" ,
			"-Wl,-rpath," .. path.getabsolute("lib")
		}
		targetdir ( "lib" )
		
	configuration { "macosx" }
        buildoptions { "-stdlib=libc++" }
		linkoptions { 
			"-stdlib=libc++" ,
			"-Wl,-rpath," .. path.getabsolute("lib")
		}
		links {
	        "OpenGL.framework",
        }
        targetdir ( "lib" )
      
	if os.get() == "macosx" then
		premake.gcc.cc = "clang"
		premake.gcc.cxx = "clang++"
		-- buildoptions("-std=c++0x -ggdb -stdlib=libc++" )
	end

	if  os.get() == "linux" then
		premake.dotnet = "mono"
	end


project "steeroptRun"
	language "C++"
	kind "ConsoleApp"
	-- kind "StaticLib"
	
	includedirs { 
		"./boost/",
	}
	files { 
		-- "include/*.h",
		-- "include/*.hpp",
		"Intersection.cpp",
		-- "src/*.cpp",
		-- "../external/libcmaes/src/esostrategy.cc"
	}
	links { 		
	}
	

	targetdir ( "bin" )

	-- linux library cflags and libs
	configuration { "linux" }
		buildoptions { 
			"`pkg-config --cflags gl`",
			"`pkg-config --cflags glu`",
			"-std=c++0x -ggdb"
		}
		linkoptions { 
			"-Wl,-rpath," .. path.getabsolute("lib") ,
			-- "`pkg-config --libs gl`",
			-- "`pkg-config --libs glu`",
			-- " -lgsl -lgslcblas" 
		}
		libdirs { "lib" }
		links {
			"dl",
			"pthread",
		}

	-- windows library cflags and libs
	configuration { "windows" }
		-- libdirs { "lib" }
		-- files {
		-- 	"../external/tinyxml/*.h",
		-- 	"../external/tinyxml/*.cpp" 
		-- }
		links { 
			"opengl32",
			"glu32",
		}

	-- mac includes and libs
	configuration { "macosx" }
		kind "ConsoleApp" -- xcode4 failes to run the project if using WindowedApp
		-- includedirs { "/Library/Frameworks/SDL.framework/Headers" }
		buildoptions { "-std=c++0x -ggdb" }
		linkoptions { 
			"-Wl,-rpath," .. path.getabsolute("lib") ,
		}
		links { 
			"OpenGL.framework", 
			"Cocoa.framework",
			-- "tinyxml",
			"dl",
			"pthread"
		}
