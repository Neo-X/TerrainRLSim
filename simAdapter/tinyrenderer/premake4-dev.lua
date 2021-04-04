local linuxLibraryLoc = "../external/"
local windowsLibraryLoc = "../../library/"

function file_exists(name)
   local f=io.open(name,"r")
   if f~=nil then io.close(f) return true else return false end
end

project "tinyRenderer"
	language "C++"
	kind "SharedLib"

	targetdir ( "../../lib" )
--	targetname ("_terrainRLAdapter")
--	targetprefix ("")
	files { 
		-- Source files for this project
		-- "render/*.cpp",
		-- "util/*.cpp",
		-- "sim/*.cpp",
		-- "anim/*.cpp",
		-- "learning/*.cpp",
		-- "scenarios/*.cpp",
		"./*.cpp",
		-- "Main.cpp"
	}
	excludes 
	{
		"main.cpp",
	}	
	includedirs { 
		"./"
	}
	links {
	}

	defines {
	}

	-- targetdir "./"
	buildoptions("-std=c++0x -O3 -ggdb -fPIC" )	

	-- linux library cflags and libs
	configuration { "linux", "gmake" }
		buildoptions { 
			" -fPIC",
		}
		linkoptions { 
			"-Wl,-rpath," .. path.getabsolute("../../lib") ,
			" -fPIC",
		}
		libdirs { 
			-- "lib",
		}
		
		includedirs { 
		}
		defines {
			"_LINUX_",
		}
			-- debug configs
		configuration { "linux", "Debug*", "gmake"}
			links {
			}
	 
	 	-- release configs
		configuration { "linux", "Release*", "gmake"}
			defines { "NDEBUG" }
			links {
			}

	-- windows library cflags and libs
	configuration { "windows" }
		-- libdirs { "lib" }
		linkoptions  { 
			-- "`pkg-config --cflags glu`" 
		}
		defines {
			"_USE_MATH_DEFINES"	
		}
		includedirs { 
		}	
		
		libdirs { 
		}
		
		-- release configs
		configuration { "windows", "Debug*"}
			defines { "DEBUG" }
			links { 
			}

		-- release configs
		configuration { "windows", "Release*"}
			defines { "NDEBUG" }
			links { 
			}

	-- mac includes and libs
	configuration { "macosx" }
		kind "ConsoleApp" -- xcode4 failes to run the project if using WindowedApp
		-- includedirs { "/Library/Frameworks/SDL.framework/Headers" }
		buildoptions { "-Wunused-value -Wshadow -Wreorder -Wsign-compare -Wall" }
		linkoptions { 
			"-Wl,-rpath," .. path.getabsolute("lib") ,
		}
		links { 
			"OpenGL.framework", 
			"Cocoa.framework",
			"dl",
			"pthread"
		}


project "tinyRendererMain"
	language "C++"
	kind "ConsoleApp"

	targetdir ( "../../lib" )
--	targetname ("_terrainRLAdapter")
--	targetprefix ("")
	files { 
		-- Source files for this project
		-- "render/*.cpp",
		-- "util/*.cpp",
		-- "sim/*.cpp",
		-- "anim/*.cpp",
		-- "learning/*.cpp",
		-- "scenarios/*.cpp",
		"./main.cpp",
		-- "Main.cpp"
	}
	includedirs { 
		"./"
	}
	links {
	}

	defines {
	}

	-- targetdir "./"
	buildoptions("-std=c++14 -O3 -ggdb -fPIC" )	

	-- linux library cflags and libs
	configuration { "linux", "gmake" }
		buildoptions { 
			" -fPIC",
		}
		linkoptions { 
			"-Wl,-rpath," .. path.getabsolute("../../lib") ,
			" -fPIC",
		}
		libdirs { 
			-- "lib",
		}
		
		includedirs { 
		}
		defines {
			"_LINUX_",
		}
			-- debug configs
		configuration { "linux", "Debug*", "gmake"}
			links {
                "tinyRenderer"
			}
	 
	 	-- release configs
		configuration { "linux", "Release*", "gmake"}
			defines { "NDEBUG" }
			links {
                "tinyRenderer"
			}

	-- windows library cflags and libs
	configuration { "windows" }
		-- libdirs { "lib" }
		linkoptions  { 
			-- "`pkg-config --cflags glu`" 
		}
		defines {
			"_USE_MATH_DEFINES"	
		}
		includedirs { 
            "./"
		}	
		
		libdirs { 
		}
		
		-- release configs
		configuration { "windows", "Debug*"}
			defines { "DEBUG" }
			links { 
			}

		-- release configs
		configuration { "windows", "Release*"}
			defines { "NDEBUG" }
			links { 
			}

	-- mac includes and libs
	configuration { "macosx" }
		kind "ConsoleApp" -- xcode4 failes to run the project if using WindowedApp
		-- includedirs { "/Library/Frameworks/SDL.framework/Headers" }
		buildoptions { "-Wunused-value -Wshadow -Wreorder -Wsign-compare -Wall" }
		linkoptions { 
			"-Wl,-rpath," .. path.getabsolute("lib") ,
		}
		links { 
			"OpenGL.framework", 
			"Cocoa.framework",
			"dl",
			"pthread"
		}


