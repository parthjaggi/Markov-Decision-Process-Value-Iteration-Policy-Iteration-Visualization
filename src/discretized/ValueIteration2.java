package discretized;

import com.google.gson.Gson;
import java.util.*;
import java.util.Arrays;
import java.awt.Dimension;


import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;

import lpsolve.*;
import discretized.Domain.TrafficEnv;
import discretized.Domain.ReservoirEnv;
import discretized.Domain.BandwidthEnv;
import discretized.Domain.Environment;

/**
 * @author GOH KA HIAN NANYANG TECHNOLOGICAL UNIVERSITY
 *
 */
public class ValueIteration2 implements Constant {
	private int iterationCount;
	public Environment env;

	// MAXIMUM CHANGE IN UTILITY
	private double maximumChange;
	// MAXIMUM ERROR ALLOWED FOR ALGORITHM, DEFAULT 0.1
	static double maximumErrorAllowed = 0.1;

	// Set the domain
	String _domain;


	// CONSTRUCTOR
	public ValueIteration2(Environment env, String domain) {
		if (domain.equalsIgnoreCase("traffic")){
			this.env = (TrafficEnv)env;
		}
		else if (domain.equalsIgnoreCase("reservoir")){
			this.env = (ReservoirEnv)env;
		}
		else if (domain.equalsIgnoreCase("bandwidth")){
			this.env = (BandwidthEnv)env;
			((BandwidthEnv) this.env).setupBandwidth();
		}
		else{
			System.out.println("Domain not recognized!");
			System.exit(1);
		}
	}

	// START VALUE ITERATION
	public void startIteration(int max_iteration) {
		iterationCount = 0;
		// discretization = env.getDiscretization();
		// max_value = env.getMaxValue();
		// ratio = env.getRatio();

		// INITIALIZE AUXILLARY ARRAY
		// this.oldUtility = new float[discretization][discretization][discretization][discretization][discretization];
		
		// INITIAL STEP: SET U(S) = 0
		// setUtlityZero();
		env.setUtilityZero();

		long startTime = System.currentTimeMillis();

		// ITERATE UNTIL CONVERGENCE BASED ON maximumErrorAllowed
		do {
			// if (iterationCount == max_iteration){
			// 	break;
			// }

			iterationCount++;
			// MAXIMUM CHANGES IN UTILITY
			maximumChange = 0;
			// KEEP TRACK OLD UTILITY
			env.storeOldUtility();
			
			// UPDATE NEW UTILITY VALUE
			env.updateUtility();

			// CALCULATE MAXIMUM CHANGES IN UTILITY
			maximumChange = env.computeMaximumDifference();
		} while (iterationCount != max_iteration); 
		//((maximumChange) >= (maximumErrorAllowed * (1.0 - DISCOUNT) / DISCOUNT) && !(maximumChange == 0));

		long endTime = System.currentTimeMillis();
		long elapsedTime = endTime - startTime;
		System.out.println("elapsed time: " + elapsedTime + "ms");

		// Get the Java runtime and Run the garbage collector
		Runtime runtime = Runtime.getRuntime();
		runtime.gc();
		long memory = runtime.totalMemory() - runtime.freeMemory();
		System.out.println("end memory: " + memory + "bytes");

		System.out.println("VI has converged (iteration: " + iterationCount + ")");

		// SAVE TO DISK
		env.storeOldUtility();
		String oldUtilityJson = env.toJson();

		try {
			String fileName = "./results/" + env._domain + "_" + env.getDiscretization() + ".txt";
			BufferedWriter writer = new BufferedWriter(new FileWriter(fileName));
			writer.write(oldUtilityJson);
			writer.close();
		} catch (IOException e) {
			e.printStackTrace();
		}
	}
}
