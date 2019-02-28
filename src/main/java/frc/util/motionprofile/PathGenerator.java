package frc.util.motionprofile;

public class PathGenerator {
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

		public static double Curvature(double[][] path) {
			if(path[0][0] == path[1][0]) {
				path[0][1] += 0.001;
			}

			if((path[0][0] - path[1][0]) * (path[1][1] - path[2][1]) - (path[1][0] - path[2][0]) * (path[0][1] - path[1][1]) == 0) {
				return 0;
			}

			double[][] minor11, minor12, minor13, minor14;
			minor11 = new double[][] {
				{path[0][0], path[0][1], 1},
				{path[1][0], path[1][1], 1},
				{path[2][0], path[2][1], 1},
			};
			minor12 = new double[][] {
				{Magnitude(path[0][0], path[0][1], 0, 0) * Magnitude(path[0][0], path[0][1], 0, 0), path[0][1], 1},
				{Magnitude(path[1][0], path[1][1], 0, 0) * Magnitude(path[1][0], path[1][1], 0, 0), path[1][1], 1},
				{Magnitude(path[2][0], path[2][1], 0, 0) * Magnitude(path[2][0], path[2][1], 0, 0), path[2][1], 1},
			};
			minor13 = new double[][] {
				{Magnitude(path[0][0], path[0][1], 0, 0) * Magnitude(path[0][0], path[0][1], 0, 0), path[0][0], 1},
				{Magnitude(path[1][0], path[1][1], 0, 0) * Magnitude(path[1][0], path[1][1], 0, 0), path[1][0], 1},
				{Magnitude(path[2][0], path[2][1], 0, 0) * Magnitude(path[2][0], path[2][1], 0, 0), path[2][0], 1},
			};
			minor14 = new double[][] {
				{Magnitude(path[0][0], path[0][1], 0, 0) * Magnitude(path[0][0], path[0][1], 0, 0), path[0][0], path[0][1]},
				{Magnitude(path[1][0], path[1][1], 0, 0) * Magnitude(path[1][0], path[1][1], 0, 0), path[1][0], path[1][1]},
				{Magnitude(path[2][0], path[2][1], 0, 0) * Magnitude(path[2][0], path[2][1], 0, 0), path[2][0], path[2][1]},
			};

			double h, k;
			h = 0.5 * Determinant33(minor12) / Determinant33(minor11);
			k = 0.5 * Determinant33(minor13) / Determinant33(minor11);

			return 1 / Math.sqrt(h * h + k * k + Determinant33(minor14) / Determinant33(minor11));
		}

		public static double maxVelocity(double pathMaxVel, double[][] path) {
			return Math.min(pathMaxVel, 2.5 / Curvature(path));
		}
}