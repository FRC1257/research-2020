package frc.util.motionprofile;

public class PathGenerator {
	public double[][] path;
	public double pathMaxVel;
	public double accel;
	public double[] segV;
	public double[][] robotPos;
	public double[][] currentLookaheadPoint;
	public double lookaheadRadius;
	public int prevClosestPoint;

	public PathGenerator(double[][] path, double pathMaxVel, double accel, double lookaheadRadius) {
		this.path = smoother(path, 0.87, 0.13, 0.001);
		this.pathMaxVel = pathMaxVel;
		this.accel = accel;
		this.robotPos = new double[][] {
			{0.0, 0.0},
		};
		this.lookaheadRadius = lookaheadRadius;
		this.currentLookaheadPoint = new double[][] {
			{this.lookaheadRadius * (this.path[1][0] - this.path[0][0]), this.lookaheadRadius * (this.path[1][1] - this.path[0][1])},
		};
		this.prevClosestPoint = 0;

		segV = segVelocity(path, accel, pathMaxVel);
	}
    /**
	 * Optimization algorithm, which optimizes the data points in path to create a smooth trajectory.
	 * This optimization uses gradient descent. While unlikely, it is possible for this algorithm to never
	 * converge. If this happens, try increasing the tolerance level.
	 * 
	 * BigO: N^x, where X is the number of of times the while loop iterates before tolerance is met. 
	 * 
	 * @param path
	 * @param weight_data
	 * @param weight_smooth
	 * @param tolerance
	 * @return
	 */
	public double[][] smoother(double[][] path, double weight_data, double weight_smooth, double tolerance) {
        
        //copy array
		double[][] newPath = doubleArrayCopy(path);

		double change = tolerance;
		while(change >= tolerance)
		{
			change = 0.0;
			for(int i=1; i<path.length-1; i++)
				for(int j=0; j<path[i].length; j++)
				{
					double aux = newPath[i][j];
					newPath[i][j] += weight_data * (path[i][j] - newPath[i][j]) + weight_smooth * (newPath[i-1][j] + newPath[i+1][j] - (2.0 * newPath[i][j]));
					change += Math.abs(aux - newPath[i][j]);	
				}					
		}

		return newPath;
    }

	/**
	 * Performs a deep copy of a 2 Dimensional Array looping thorough each element in the 2D array
	 * 
	 * BigO: Order N x M
	 * @param arr
	 * @return
	 */
	public static double[][] doubleArrayCopy(double[][] arr) {

		//size first dimension of array
		double[][] temp = new double[arr.length][arr[0].length];

		for(int i=0; i<arr.length; i++)
		{
			//Resize second dimension of array
			temp[i] = new double[arr[i].length];

			//Copy Contents
			for(int j=0; j<arr[i].length; j++)
				temp[i][j] = arr[i][j];
		}

		return temp;
		}
		
		public static double Magnitude(double x1, double y1, double x2, double y2) {
			return Math.sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
		}

		public static double Determinant33(double[][] matrix) {
			double det1, det2, det3;
			det1 = matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1];
			det2 = matrix[1][0] * matrix[2][2] - matrix[1][2] * matrix[2][0];
			det3 = matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0];
			return det1 - det2 + det3;
		}

		public static double[] curvature(double[][] path) {
			double[] result;
			result = new double[path.length];

			for(int i = 0; i < path.length - 2; ++i) {
				if(path[i + 0][0] == path[i + 1][0]) {
					path[i + 0][1] += 0.001;
				}

				if((path[i + 0][0] - path[i + 1][0]) * (path[i + 1][1] - path[i + 2][1]) - (path[i + 1][0] - path[i + 2][0]) * (path[i + 0][1] - path[i + 1][1]) == 0) {
					result[i + 1] = 0.0;
				} else {
					double[][] minor11, minor12, minor13, minor14;
					minor11 = new double[][] {
						{path[i + 0][0], path[i + 0][1], 1},
						{path[i + 1][0], path[i + 1][1], 1},
						{path[i + 2][0], path[i + 2][1], 1},
					};
					minor12 = new double[][] {
						{Magnitude(path[i + 0][0], path[i + 0][1], 0, 0) * Magnitude(path[i + 0][0], path[i + 0][1], 0, 0), path[i + 0][1], 1},
						{Magnitude(path[i + 1][0], path[i + 1][1], 0, 0) * Magnitude(path[i + 1][0], path[i + 1][1], 0, 0), path[i + 1][1], 1},
						{Magnitude(path[i + 2][0], path[i + 2][1], 0, 0) * Magnitude(path[i + 2][0], path[i + 2][1], 0, 0), path[i + 2][1], 1},
					};
					minor13 = new double[][] {
						{Magnitude(path[i + 0][0], path[i + 0][1], 0, 0) * Magnitude(path[i + 0][0], path[i + 0][1], 0, 0), path[i + 0][0], 1},
						{Magnitude(path[i + 1][0], path[i + 1][1], 0, 0) * Magnitude(path[i + 1][0], path[i + 1][1], 0, 0), path[i + 1][0], 1},
						{Magnitude(path[i + 2][0], path[i + 2][1], 0, 0) * Magnitude(path[i + 2][0], path[i + 2][1], 0, 0), path[i + 2][0], 1},
					};
					minor14 = new double[][] {
						{Magnitude(path[i + 0][0], path[i + 0][1], 0, 0) * Magnitude(path[i + 0][0], path[i + 0][1], 0, 0), path[i + 0][0], path[i + 0][1]},
						{Magnitude(path[i + 1][0], path[i + 1][1], 0, 0) * Magnitude(path[i + 1][0], path[i + 1][1], 0, 0), path[i + 1][0], path[i + 1][1]},
						{Magnitude(path[i + 2][0], path[i + 2][1], 0, 0) * Magnitude(path[i + 2][0], path[i + 2][1], 0, 0), path[i + 2][0], path[i + 2][1]},
					};
					double h, k;
					h = 0.5 * Determinant33(minor12) / Determinant33(minor11);
					k = 0.5 * Determinant33(minor13) / Determinant33(minor11);

					result[i + 1] =  1 / Math.sqrt(h * h + k * k + Determinant33(minor14) / Determinant33(minor11));
				};
				if(i == 0 || i == path.length - 1) {
					result[i] = 0;
				}
			};
				return result;
		}

		public static double[] maxVelocity(double pathMaxVel, double[][] path) {
			double[] result;
			result = new double[path.length];
			for(int i = 1; i < path.length - 1; ++i) {
				if(curvature(path)[i] == 0) {
					result[i] = pathMaxVel;
				} else {
					result[i] = Math.min(pathMaxVel, 2.5 / curvature(path)[i]);
				}
			}
			result[0] = pathMaxVel;
			result[path.length - 1] = pathMaxVel;
			return result;
		}

		public static double[] segVelocity(double[][] path, double accel, double pathMaxVel) {
			double[] result;
			result = new double[path.length];
			result[path.length - 1] = 0.0;
			for(int i = path.length - 1; i > -1; i = i - 1) {
				result[i] = Math.min(maxVelocity(pathMaxVel, path)[i], Math.sqrt(result[i + 1] * result[i + 1] - 2 * accel * Magnitude(path[i][0], path[i][1], path[i + 1][0], path[i + 1][1])));
			}
			return result;
		}

		public int closestPoint(double[][] robotPos) {
			double[][] point;
			point = new double[][] {
				{this.path[0][0], this.path[0][1]},
			};
			for(int i = 0; i < this.path.length; ++i) {
				if(Magnitude(robotPos[0][0], robotPos[0][1], this.path[i][0], this.path[i][1]) < Magnitude(robotPos[0][0], robotPos[0][1], point[0][0], point[0][1])) {
					point[0][0] = this.path[i][0];
					point[0][1] = this.path[i][1];
					this.prevClosestPoint = i;
					return this.prevClosestPoint;
				}
			}
			return this.prevClosestPoint;
		}

		public double[][] lookaheadPoint(double[][] robotPos) {
			double[][] point;
			point = new double[][] {
				{this.currentLookaheadPoint[0][0], this.currentLookaheadPoint[0][1]},
				// fracIndex, index
				{this.currentLookaheadPoint[1][0], this.currentLookaheadPoint[1][1]},
			};
			int index = (int) Math.round(this.currentLookaheadPoint[1][1]);
			for(int i = index; i < this.path.length - 1; ++i) {
				double[][] d = new double[][] {
					{this.path[i + 1][0] - this.path[i][0], this.path[i + 1][1] - this.path[i][1]},
				};
				double[][] f = new double[][] {
					{this.path[i][0] - robotPos[0][0], this.path[i][1] - robotPos[0][1]},
				};
				double a = Magnitude(d[0][0], d[0][1], 0.0, 0.0) * Magnitude(d[0][0], d[0][1], 0.0, 0.0);
				double b = 2 * (d[0][0] * f[0][0] + d[0][1] * f[0][1]);
				double c = Magnitude(f[0][0], f[0][1], 0.0, 0.0) * Magnitude(f[0][0], f[0][1], 0.0, 0.0) - this.lookaheadRadius * this.lookaheadRadius;
				double discriminant = b * b - 4 * a * c;

				if(discriminant < 0) {
				} else {
					discriminant = Math.sqrt(discriminant);
					double t1 = (-b - discriminant) / (2 * a);
					double t2 = (-b + discriminant) / (2 * a);

					if(t1 >= 0.0 && t1 <= 1.0) {
						t2 = 0;
					} else if(t2 >= 0.0 && t2 <= 1.0) {
						t1 = 0;
					}
					
					point[0][0] = this.path[i][0] + (t1 + t2) * d[0][0];
					point[0][1] = this.path[i][1] + (t1 + t2) * d[0][1];
					point[1][0] = t1 + t2 + i;
					if(point[1][0] > this.currentLookaheadPoint[1][1]) {
						point[1][1] = i;
						this.currentLookaheadPoint = point;
						return point;
					}
				}
			};
			return point;
		}

		public double lookaheadCurvature(double[][] robotPos) {
			double dx = this.currentLookaheadPoint[0][0] - robotPos[0][0];
			double dy = this.currentLookaheadPoint[0][1] - robotPos[0][1];

			double lookaheadDistance = Magnitude(this.currentLookaheadPoint[0][0], this.currentLookaheadPoint[0][1], robotPos[0][0], robotPos[0][1]);
			double sin = dx / lookaheadDistance;
			double cos = dy / lookaheadDistance;
			double tan = dy / dx;

			double a = -1 * tan;
			double c = tan * robotPos[0][0] - robotPos[0][1];
			double x = Math.abs(a * this.currentLookaheadPoint[0][0] + this.currentLookaheadPoint[0][1] + c) / Math.sqrt(a * a + 1);

			int sign = 1;
			if(sin * (this.currentLookaheadPoint[0][0] - robotPos[0][0]) - cos * (this.currentLookaheadPoint[0][1] - robotPos[0][1]) < 0) {
				sign -= 2;
			}

			return 2 * sign * x / (lookaheadDistance * lookaheadDistance);
		}

		public double[] velocities(double trackWidth, double[][] robotPos) {
			double[] speed = new double[2];
			double c = lookaheadCurvature(this.robotPos);
			double v = segV[closestPoint(robotPos)];
			speed[0] = v * (2 + c * trackWidth) / 2;
			speed[1] = v * (2 - c * trackWidth) / 2;

			return speed;
		}

		public void updatePos(double xCoord, double yCoord) {
			this.robotPos[0][0] = xCoord;
			this.robotPos[0][1] = yCoord;
		}
}