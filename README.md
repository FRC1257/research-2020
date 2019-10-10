# Pure Pursuit

[![Build Status](https://travis-ci.com/FRC1257/research-2020.svg?branch=PurePursuit)](https://travis-ci.com/FRC1257/research-2020)

Motion profiling++.

### Changelog

* March 24, 2019: completed the first full version of the motion profiler. Still needs to be tested on Processing, but given an array of points the math works out.
* April 11, 2019: added quintic spline interpolation. This is to be used instead of a path injector and smoother. 
* April 12, 2019: added quintic spline functionality to the path generator.
* September 20, 2019: revisited the project. Made it compatible with WPILib 4.1.
* September 24, 2019: I learned the hard way that `int` divided by `int` returns an `int` and not a `double`, and that screwed over half my code. So I fixed that. Anyhow, the spline works as expected. Also, `Math.floor()` returns a double for some reason. 
* September 26, 2019: more `int` divided by `int` issues. Honestly unproductive.
* October 1, 2019: Fixed `PathGenerator` to decrease `lookaheadRadius` by 0.1 if it doesn't find a lookahead point.
* October 7, 2019: Triple-checked `PathGenerator` and `Spline`. It works.
* October 10, 2019: I have no idea what WPILib is doing with their documentation. But I made `currentLookaheadPoint` easier to find and use in the methods. Once WPILib gets their documentation straight I'll do restructure of the command statements. I'll prepare to start PID tuning soon.
