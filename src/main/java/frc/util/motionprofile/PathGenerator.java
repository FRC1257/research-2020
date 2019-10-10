package frc.util.motionprofile;

import frc.util.motionprofile.Spline;

/**
 * <h1>PathGenerator</h1>
 * Generates a motion-profiled pure pursuit path given a path and some robot parameters. 
 * The path is a 2-dimensional array (n rows by 2 columns), with each point having an x and y coordinate.
 * <b>UNITS ARE IN INCHES.</b>
 * 
 * @author Allen Du
 * @since 2019-03-10
 */
public class PathGenerator {
	public double[][] path;
	public double pathMaxVel;
	public double accel;
	public double[] segV;
	public double[][] robotPos;
	public double[][] currentLookaheadPoint;
	// Why is this 12 when the recommended distance between points is 6?
	public double lookaheadRadius;
	public int prevClosestPoint;

	/**
	 * The constructor for a {@code PathGenerator} object.
	 * <b>UNITS ARE IN INCHES.</b>
	 * 
	 * @param path The path, represented by a spline.
	 * @param pathMaxVel The maxmimum theoretical velocity the path can be.
	 * @param accel The acceleration of the robot.
	 * @param lookaheadRadius The radius for the lookahead point. Usually a value from 12-25 inches.
	 */
	public PathGenerator(Spline path, double pathMaxVel, double accel, double lookaheadRadius) {
		// // Doesn't have to be 6, but it is 6.
		// this.path = inject(6.0);
		// // May change the second and third values depending on the path.
		// this.path = smoother(this.path, 0.87, 0.13, 0.001);
		double[][] temp = path.interpolate(0.01, 6.0);
		for(int i = 0; i < temp.length; ++i) {
			this.path[i][0] = temp[i][0];
			this.path[i][1] = temp[i][1];
		}
		this.pathMaxVel = pathMaxVel;
		this.accel = accel;
		this.robotPos = new double[][] {
			{0.0, 0.0},
		};
		this.lookaheadRadius = lookaheadRadius;
		// The first lookahead point is just a point along the first. I feel like this is wrong.
		this.currentLookaheadPoint = new double[][] {
			{this.lookaheadRadius * (this.path[1][0] - this.path[0][0]), this.lookaheadRadius * (this.path[1][1] - this.path[0][1])},
			{0.001, 0.0},
		};
		this.prevClosestPoint = 0;

		this.segV = new double[this.path.length];
		this.segV[this.path.length - 1] = 0.0;
		for(int i = this.path.length - 2; i >= 0; i--) {
			double[][] temp2 = new double[][] {
				{this.path[i - 1][0], this.path[i - 1][1]},
				{this.path[i][0], this.path[i][1]},
				{this.path[i + 1][0], this.path[i + 1][1]},
			};
			this.segV[i] = Math.min(maxVelocity(temp2), Math.sqrt(this.segV[i + 1] * this.segV[i + 1] - 2 * this.accel * Magnitude(this.path[i + 1][0], this.path[i + 1][1], this.path[i][0], this.path[i][1])));
		}
	}

	// /**
	//  * Point injector. It injects evenly spaced points along a given path.
	//  * 
	//  * @return This is the path used in the smoother.
	//  */
	// public double[][] inject(double spacing) {
	// 	int totalPointsThatFit = 0;

	// 	for(int i = 0; i < this.path.length - 1; ++i) {
	// 		double[][] vector = new double[][] {
	// 			{this.path[i + 1][0] - this.path[i][0], this.path[i + 1][1] - this.path[i][1]},
	// 		};

	// 		int pointsThatFit = (int) Math.ceil(Math.sqrt(vector[0][0] * vector[0][0] + vector[0][1] * vector[0][1]) / spacing);
	// 		totalPointsThatFit += pointsThatFit;
	// 	}
	// 	// System.out.println("Total fitting points: " + totalPointsThatFit);
	// 	double[][] result = new double[totalPointsThatFit + 1][2];

	// 	for(int j = 0; j < this.path.length - 1; ++j) {
	// 		double[][] vector = new double[][] {
	// 			{this.path[j + 1][0] - this.path[j][0], this.path[j + 1][1] - this.path[j][1]},
	// 		};

	// 		int pointsThatFit = (int) Math.ceil(Math.sqrt(vector[0][0] * vector[0][0] + vector[0][1] * vector[0][1]) / spacing);

	// 		double magnitude = Math.sqrt(vector[0][0] * vector[0][0] + vector[0][1] * vector[0][1]);
	// 		vector[0][0] /= magnitude;
	// 		vector[0][0] *= spacing;
	// 		// System.out.println("Vector x: " + vector[0][0]);
	// 		vector[0][1] /= magnitude;
	// 		vector[0][1] *= spacing;
	// 		// System.out.println("Vector y: " + vector[0][1]);

	// 		// System.out.println("Points that fit: " + pointsThatFit);
	// 		for(int k = 0; k < pointsThatFit; ++k) {
	// 			result[j * pointsThatFit + k][0] = this.path[j][0] + vector[0][0] * k;
	// 			result[j * pointsThatFit + k][1] = this.path[j][1] + vector[0][1] * k;
	// 			// System.out.println("Result " + temp + ": (" + result[j * pointsThatFit + k][0] + ", " + result[j * pointsThatFit + k][1] + ")");
	// 		}
	// 	}

	// 	result[totalPointsThatFit][0] = this.path[this.path.length - 1][0];
	// 	result[totalPointsThatFit][1] = this.path[this.path.length - 1][1];

	// 	return result;
	// }
    // /**
	//  * <i>This method is by Team FRC 2168.</i>
	//  * 
	//  * Optimization algorithm, which optimizes the data points in path to create a smooth trajectory.
	//  * This optimization uses gradient descent. While unlikely, it is possible for this algorithm to never
	//  * converge. If this happens, try increasing the tolerance level.
	//  * 
	//  * BigO: N^x, where X is the number of of times the while loop iterates before tolerance is met. 
	//  * 
	//  * @param path The path.
	//  * @param weight_data A value, usually between 0.8 and 0.99.
	//  * @param weight_smooth 1 - {@code weight_data}.
	//  * @param tolerance This is 0.001, but change as you wish.
	//  * @return A smoothed path.
	//  */
	// public double[][] smoother(double[][] path, double weight_data, double weight_smooth, double tolerance) {
        
    //     //copy array
	// 	double[][] newPath = doubleArrayCopy(path);

	// 	double change = tolerance;
	// 	while(change >= tolerance)
	// 	{
	// 		change = 0.0;
	// 		for(int i=1; i<path.length-1; i++)
	// 			for(int j=0; j<path[i].length; j++)
	// 			{
	// 				double aux = newPath[i][j];
	// 				newPath[i][j] += weight_data * (path[i][j] - newPath[i][j]) + weight_smooth * (newPath[i-1][j] + newPath[i+1][j] - (2.0 * newPath[i][j]));
	// 				change += Math.abs(aux - newPath[i][j]);	
	// 			}					
	// 	}

	// 	return newPath;
    // }

	// /**
	//  * <i>This method is by FRC Team 2168.</i>
	//  * 
	//  * Performs a deep copy of a 2 Dimensional Array looping thorough each element in the 2D array
	//  * 
	//  * BigO: Order N x M
	//  * @param arr
	//  * @return A copied array.
	//  */
	// public static double[][] doubleArrayCopy(double[][] arr) {

	// 	//size first dimension of array
	// 	double[][] temp = new double[arr.length][arr[0].length];

	// 	for(int i=0; i<arr.length; i++)
	// 	{
	// 		//Resize second dimension of array
	// 		temp[i] = new double[arr[i].length];

	// 		//Copy Contents
	// 		for(int j=0; j<arr[i].length; j++)
	// 			temp[i][j] = arr[i][j];
	// 	}

	// 	return temp;
	// 	}
		
		/**
		 * Distance formula, given two points (x<sub>1</sub>, y<sub>1</sub>) and (x<sub>2</sub>, y<sub>2</sub>).
		 * 
		 * @param x1 
		 * @param y1
		 * @param x2
		 * @param y2
		 * 
		 * @return The distance.
		 */
		public static double Magnitude(double x1, double y1, double x2, double y2) {
			return Math.sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
		}

		/**
		 * Calculates the curvature of a point.
		 * 
		 * @param path The path.
		 * 
		 * @return The curvature of a given point. 
		 */
		public static double curvature(double[][] path) {
			if(path[0][0] == path[1][0]) {
				path[0][0] += 0.001;
			}
			
			double k1, k2;
			k1 = 0.5 * (path[0][0] * path[0][0] + path[0][1] * path[0][1] - path[1][0] * path[1][0] - path[1][1] * path[1][1]) / (path[0][0] - path[1][0]);
			k2 = (path[0][1] - path[1][1]) / (path[0][0] - path[1][0]);
			double a, b;
			b = 0.5 * (path[1][0] * path[1][0] - 2 * path[1][0] * k1 + path[1][1] * path[1][1] - path[2][0] * path[2][0] + 2 * path[2][0] * k1 - path[2][1] * path[2][1]) / (path[2][0] * k2 - path[2][1] + path[1][1] - path[1][0] * k2);
			a = k1 - k2 * b;
	
			// System.out.println("k1 = " + k1);
			// System.out.println("k2 = " + k2);
			// System.out.println("a = " + a);
			// System.out.println("b = " + b);
	
			double r;
			if((path[1][1] - path[0][1]) / (path[1][0] - path[0][0]) == (path[2][1] - path[1][1]) / (path[2][0] - path[1][0])) {
				r = 0.0;
			} else {
				r = Math.sqrt((path[0][0] - a) * (path[0][0] - a) + (path[0][1] - b) * (path[0][1] - b));
			}
			// System.out.println("Radius: " + r);
	
			// if((path[1][1] - path[0][1]) / (path[1][0] - path[0][0]) == (path[2][1] - path[1][1]) / (path[2][0] - path[1][0])) {
			// 	return 0.0;
			// }
			return 1 / r;
		}

		/**
		 * Calculates the maximum theoretical velocity of a point.
		 * 
		 * @param pathMaxVel The maximum path velocity.
		 * @param path 
		 * 
		 * @return The maximum theoretical velocity of a point.
		 */
		public double maxVelocity(double[][] path) {
			double result = 0.0;
				if((path[1][1] - path[0][1]) / (path[1][0] - path[0][0]) == (path[2][1] - path[1][1]) / (path[2][0] - path[1][0])) {
					// For collinear points
					result = this.pathMaxVel;
				} else {
					// Might change that 2.5 to something else between 1 and 5
					result = Math.min(this.pathMaxVel, 2.5 / curvature(path));
				};
			// System.out.println("Max velocity: " + result);
	
			return result;
		}

		/**
		 * Gives the closest path point to the robot.
		 * 
		 * @param robotPos The current robot position, in the form of a 2 row by 1 column array (x, y).
		 * 
		 * @return The index of the closest path point.
		 */
		public int closestPoint() {
			// The 0 entry is the distance; the 1 entry is the index.
			double[][] distances = new double[this.path.length - this.prevClosestPoint][2];
			// Just to make sure our robot never goes backwards
			for(int i = this.prevClosestPoint; i < this.path.length; ++i) {
				distances[i - this.prevClosestPoint][0] = Magnitude(this.robotPos[0][0], this.robotPos[0][1], this.path[i][0], this.path[i][1]);
				distances[i - this.prevClosestPoint][1] = i;
			}
			// Sort the distances in ascending order, then return the index of the shortest distance.
			quickSort(distances, 0, distances.length - 1);
			this.prevClosestPoint = (int) distances[0][1];

			return this.prevClosestPoint;
		}

		/**
		 * Gets you the lookahead point.
		 * 
		 * @return A 2*2 array, with the first row having the x and y coordinates and the second having the fractional index and the index.
		 */
		public double[][] lookaheadPoint(double objectLookaheadRadius) {
            double[][] point;
            double[][] result = this.currentLookaheadPoint;
			point = new double[][] {
				{this.currentLookaheadPoint[0][0], this.currentLookaheadPoint[0][1]},
				// fracIndex, index
				{this.currentLookaheadPoint[1][0], this.currentLookaheadPoint[1][1]},
			};
			int index = (int) Math.round(this.currentLookaheadPoint[1][1]);
			// Another search.
			for(int i = index; i < this.path.length - 1; ++i) {
				double[][] d = new double[][] {
					{this.path[i + 1][0] - this.path[i][0], this.path[i + 1][1] - this.path[i][1]},
				};
				double[][] f = new double[][] {
					{this.path[i][0] - this.robotPos[0][0], this.path[i][1] - this.robotPos[0][1]},
				};
				double a = Magnitude(d[0][0], d[0][1], 0.0, 0.0) * Magnitude(d[0][0], d[0][1], 0.0, 0.0);
				double b = 2 * (d[0][0] * f[0][0] + d[0][1] * f[0][1]);
				double c = Magnitude(f[0][0], f[0][1], 0.0, 0.0) * Magnitude(f[0][0], f[0][1], 0.0, 0.0) - objectLookaheadRadius * objectLookaheadRadius;
				double discriminant = b * b - 4 * a * c;

				if(discriminant < 0) {
					return lookaheadPoint(objectLookaheadRadius - 0.1);
				} else {
					discriminant = Math.sqrt(discriminant);
					double t1 = (-b - discriminant) / (2 * a);
					double t2 = (-b + discriminant) / (2 * a);

					if(t1 >= 0.0 && t1 <= 1.0) {
						t2 = 0;
					} else if(t2 >= 0.0 && t2 <= 1.0) {
						t1 = 0;
					} else {
                        continue;
                    }
					
					point[0][0] = this.path[i][0] + (t1 + t2) * d[0][0];
					point[0][1] = this.path[i][1] + (t1 + t2) * d[0][1];
					point[1][0] = t1 + t2 + i;
					if(point[1][0] > this.currentLookaheadPoint[1][1]) {
						point[1][1] = i;
                        this.currentLookaheadPoint = point;
                        result = point;

						return result;
					}
				}
            };
            result = point;

			return result;
		}
		
		// /**
		//  * Gets you the y-coordinate of the lookahead point.
		//  * 
		//  * @return y-coordinate
		//  */
		// public double lookaheadPointY() {
        //     double[][] point;
        //     double result = this.currentLookaheadPoint[0][0];
		// 	point = new double[][] {
		// 		{this.currentLookaheadPoint[0][0], this.currentLookaheadPoint[0][1]},
		// 		// fracIndex, index
		// 		{this.currentLookaheadPoint[1][0], this.currentLookaheadPoint[1][1]},
		// 	};
		// 	int index = (int) Math.round(this.currentLookaheadPoint[1][1]);
		// 	// Another search.
		// 	for(int i = index + 1; i < this.path.length - 1; ++i) {
		// 		double[][] d = new double[][] {
		// 			{this.path[i + 1][0] - this.path[i][0], this.path[i + 1][1] - this.path[i][1]},
		// 		};
		// 		double[][] f = new double[][] {
		// 			{this.path[i][0] - this.robotPos[0][0], this.path[i][1] - this.robotPos[0][1]},
		// 		};
		// 		double a = Magnitude(d[0][0], d[0][1], 0.0, 0.0) * Magnitude(d[0][0], d[0][1], 0.0, 0.0);
		// 		double b = 2 * (d[0][0] * f[0][0] + d[0][1] * f[0][1]);
		// 		double c = Magnitude(f[0][0], f[0][1], 0.0, 0.0) * Magnitude(f[0][0], f[0][1], 0.0, 0.0) - this.lookaheadRadius * this.lookaheadRadius;
		// 		double discriminant = b * b - 4 * a * c;

		// 		if(discriminant < 0) {
		// 		} else {
		// 			discriminant = Math.sqrt(discriminant);
		// 			double t1 = (-b - discriminant) / (2 * a);
		// 			double t2 = (-b + discriminant) / (2 * a);

		// 			if(t1 >= 0.0 && t1 <= 1.0) {
		// 				t2 = 0;
		// 			} else if(t2 >= 0.0 && t2 <= 1.0) {
		// 				t1 = 0;
		// 			} else {
        //                 continue;
        //             }
					
		// 			point[0][0] = this.path[i][0] + (t1 + t2) * d[0][0];
		// 			point[0][1] = this.path[i][1] + (t1 + t2) * d[0][1];
		// 			point[1][0] = t1 + t2 + i;
		// 			if(point[1][0] > this.currentLookaheadPoint[1][1]) {
		// 				point[1][1] = i;
        //                 this.currentLookaheadPoint = point;
        //                 result = point[0][0];

		// 				return result;
		// 			}
		// 		}
        //     };
        //     result = point[0][1];

		// 	return result;
		// }
		
		// /**
		//  * @return the fractional index of the lookahead point.
		//  */
		// public double lookaheadPointFracIndex() {
        //     double[][] point;
        //     double result = this.currentLookaheadPoint[0][0];
		// 	point = new double[][] {
		// 		{this.currentLookaheadPoint[0][0], this.currentLookaheadPoint[0][1]},
		// 		// fracIndex, index
		// 		{this.currentLookaheadPoint[1][0], this.currentLookaheadPoint[1][1]},
		// 	};
		// 	int index = (int) Math.round(this.currentLookaheadPoint[1][1]);
		// 	// Another search.
		// 	for(int i = index + 1; i < this.path.length - 1; ++i) {
		// 		double[][] d = new double[][] {
		// 			{this.path[i + 1][0] - this.path[i][0], this.path[i + 1][1] - this.path[i][1]},
		// 		};
		// 		double[][] f = new double[][] {
		// 			{this.path[i][0] - this.robotPos[0][0], this.path[i][1] - this.robotPos[0][1]},
		// 		};
		// 		double a = Magnitude(d[0][0], d[0][1], 0.0, 0.0) * Magnitude(d[0][0], d[0][1], 0.0, 0.0);
		// 		double b = 2 * (d[0][0] * f[0][0] + d[0][1] * f[0][1]);
		// 		double c = Magnitude(f[0][0], f[0][1], 0.0, 0.0) * Magnitude(f[0][0], f[0][1], 0.0, 0.0) - this.lookaheadRadius * this.lookaheadRadius;
		// 		double discriminant = b * b - 4 * a * c;

		// 		if(discriminant < 0) {
		// 		} else {
		// 			discriminant = Math.sqrt(discriminant);
		// 			double t1 = (-b - discriminant) / (2 * a);
		// 			double t2 = (-b + discriminant) / (2 * a);

		// 			if(t1 >= 0.0 && t1 <= 1.0) {
		// 				t2 = 0;
		// 			} else if(t2 >= 0.0 && t2 <= 1.0) {
		// 				t1 = 0;
		// 			} else {
        //                 continue;
        //             }
					
		// 			point[0][0] = this.path[i][0] + (t1 + t2) * d[0][0];
		// 			point[0][1] = this.path[i][1] + (t1 + t2) * d[0][1];
		// 			point[1][0] = t1 + t2 + i;
		// 			if(point[1][0] > this.currentLookaheadPoint[1][1]) {
		// 				point[1][1] = i;
        //                 this.currentLookaheadPoint = point;
        //                 result = point[1][0];

		// 				return result;
		// 			}
		// 		}
        //     };
        //     result = point[1][0];

		// 	return result;
		// }
		
		// /**
		//  * @return the index of the lookahead point; or rather, the startpoint of the line segment the lookahead point is on.
		//  */
		// public double lookaheadPointIndex() {
        //     double[][] point;
        //     double result = this.currentLookaheadPoint[0][0];
		// 	point = new double[][] {
		// 		{this.currentLookaheadPoint[0][0], this.currentLookaheadPoint[0][1]},
		// 		// fracIndex, index
		// 		{this.currentLookaheadPoint[1][0], this.currentLookaheadPoint[1][1]},
		// 	};
		// 	int index = (int) Math.round(this.currentLookaheadPoint[1][1]);
		// 	// Another search.
		// 	for(int i = index + 1; i < this.path.length - 1; ++i) {
		// 		double[][] d = new double[][] {
		// 			{this.path[i + 1][0] - this.path[i][0], this.path[i + 1][1] - this.path[i][1]},
		// 		};
		// 		double[][] f = new double[][] {
		// 			{this.path[i][0] - this.robotPos[0][0], this.path[i][1] - this.robotPos[0][1]},
		// 		};
		// 		double a = Magnitude(d[0][0], d[0][1], 0.0, 0.0) * Magnitude(d[0][0], d[0][1], 0.0, 0.0);
		// 		double b = 2 * (d[0][0] * f[0][0] + d[0][1] * f[0][1]);
		// 		double c = Magnitude(f[0][0], f[0][1], 0.0, 0.0) * Magnitude(f[0][0], f[0][1], 0.0, 0.0) - this.lookaheadRadius * this.lookaheadRadius;
		// 		double discriminant = b * b - 4 * a * c;

		// 		if(discriminant < 0) {
		// 		} else {
		// 			discriminant = Math.sqrt(discriminant);
		// 			double t1 = (-b - discriminant) / (2 * a);
		// 			double t2 = (-b + discriminant) / (2 * a);

		// 			if(t1 >= 0.0 && t1 <= 1.0) {
		// 				t2 = 0;
		// 			} else if(t2 >= 0.0 && t2 <= 1.0) {
		// 				t1 = 0;
		// 			} else {
        //                 continue;
        //             }
					
		// 			point[0][0] = this.path[i][0] + (t1 + t2) * d[0][0];
		// 			point[0][1] = this.path[i][1] + (t1 + t2) * d[0][1];
		// 			point[1][0] = t1 + t2 + i;
		// 			if(point[1][0] > this.currentLookaheadPoint[1][1]) {
		// 				point[1][1] = i;
        //                 this.currentLookaheadPoint = point;
        //                 result = point[1][1];

		// 				return result;
		// 			}
		// 		}
        //     };
        //     result = point[1][1];

		// 	return result;
		// }
		
		/**
		 * Calculates the curvature of the lookahead point.
		 * 
		 * @return If the value is positive, the lookahead point is on the right; it's on the left if otherwise.
		 */
		public double lookaheadCurvature() {
			updateLookaheadPoint();
			
			double dx = this.currentLookaheadPoint[0][0] - this.robotPos[0][0];
			double dy = this.currentLookaheadPoint[0][1] - this.robotPos[0][1];

			double lookaheadDistance = Magnitude(this.currentLookaheadPoint[0][0], this.currentLookaheadPoint[0][1], this.robotPos[0][0], this.robotPos[0][1]);
			double cos = dy / lookaheadDistance;
			double tan = dy / dx;

			double a = -1.0 * tan;
			double c = tan * this.robotPos[0][0] - this.robotPos[0][1];
			// Point-to-line
			double x = Math.abs(a * this.currentLookaheadPoint[0][0] + this.currentLookaheadPoint[0][1] + c) / Math.sqrt(a * a + 1);

			double sign = Math.signum((this.currentLookaheadPoint[0][0] - this.robotPos[0][0]) - cos * (this.currentLookaheadPoint[0][1] - this.robotPos[0][1]));

			return 2.0 * sign * x / (lookaheadDistance * lookaheadDistance);
		}

		/**
		 * Returns velocities.
		 * 
		 * @param trackWidth The distance between the left and right wheels.
		 * @param robotPos
		 * 
		 * @return The first array entry is the left; the second is the right.
		 */
		public double velocity(double trackWidth, boolean left) {
			double speed = 0.0;
			double c = lookaheadCurvature();
			double v = segV[closestPoint()];
			if(left) {
				speed = v * (2 + c * trackWidth) / 2;
			} else {
				speed = v * (2 - c * trackWidth) / 2;
			};
	
			return speed;
		}

		/**
		 * Updates the robot position.
		 * 
		 * @param xCoord
		 * @param yCoord
		 */
		public void updatePos(double xCoord, double yCoord) {
			this.robotPos[0][0] = xCoord;
			this.robotPos[0][1] = yCoord;
		}

		public void updateLookaheadPoint() {
			this.currentLookaheadPoint = lookaheadPoint(this.lookaheadRadius);
		}

		/**
		 * I copied the quicksort sorting algorithm and adjusted it for 2-dimensional arrays.
		 */
		public static void quickSort(double[][] arr, int low, int high)
    {
        //check for empty or null array
        if (arr == null || arr.length == 0){
            return;
        }
         
        if (low >= high){
            return;
        }
 
        //Get the pivot element from the middle of the list
        int middle = low + (high - low) / 2;
        double pivot = arr[middle][0];
 
        // make left < pivot and right > pivot
        int i = low, j = high;
        while (i <= j)
        {
            //Check until all values on left side array are lower than pivot
            while (arr[i][0] < pivot)
            {
                i++;
            }
            //Check until all values on left side array are greater than pivot
            while (arr[j][0] > pivot)
            {
                j--;
            }
            //Now compare values from both side of lists to see if they need swapping
            //After swapping move the iterator on both lists
            if (i <= j)
            {
                swap (arr, i, j);
                i++;
                j--;
            }
        }
        //Do same operation as above recursively to sort two sub arrays
        if (low < j){
            quickSort(arr, low, j);
        }
        if (high > i){
            quickSort(arr, i, high);
        }
    }
	 
	/**
	 * Swapping two variables.
	 */
    public static void swap (double[][] array, int x, int y)
    {
        double temp = array[x][0];
        array[x][0] = array[y][0];
        array[y][0] = temp;
	}
	
	/**
	 * @return The path.
	 */
	public double[][] getPath() {
		return this.path;
	}
}