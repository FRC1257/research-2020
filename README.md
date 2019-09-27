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