# The Hole calibration tool

Fork of [HeadSpaceRS](https://github.com/tecartlab/HeadSpaceRS) by Martin Fröhlich (under MIT license).

## Compiling

Before compiling, you need to edit the file `ofxRealSenseTwo/addon_config.mk` in your _OpenFrameworks_ addons folder in the following way : 

```makefile
linux64:
	# Add the following line in the linux64 section
	ADDON_LDFLAGS = -lrealsense2
```
