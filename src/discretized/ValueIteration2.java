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
	public long elapsedTime;
	public long memory;
	public long[] iterElapsedTime;
	public long[] iterMemory;


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
		iterElapsedTime = new long[max_iteration];
		iterMemory = new long[max_iteration];

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
		Runtime runtime = Runtime.getRuntime();

		// ITERATE UNTIL CONVERGENCE BASED ON maximumErrorAllowed
		do {
			long iterStart = System.currentTimeMillis();

			iterationCount++;
			// MAXIMUM CHANGES IN UTILITY
			maximumChange = 0;
			// KEEP TRACK OLD UTILITY
			env.storeOldUtility();
			
			// UPDATE NEW UTILITY VALUE
			env.updateUtility();

			// Save LP solution feasibility for debugging
			// String isFeasibleJson = env.toJson2();
			// String filepath = "./results/" + env._domain + "_d_" + env.getDiscretization() + "_isFeasible_a_1_r_1.txt";
			// Utils.saveStringToFile(isFeasibleJson, filepath);
			// System.exit(1);

			// CALCULATE MAXIMUM CHANGES IN UTILITY
			maximumChange = env.computeMaximumDifference();

			// Run garbage collector and record memory usage.
			runtime.gc();
			memory = runtime.totalMemory() - runtime.freeMemory();
			iterMemory[iterationCount - 1] = memory;

			long iterEnd = System.currentTimeMillis();
			iterElapsedTime[iterationCount - 1] = iterEnd - iterStart;

			System.out.println("iterationCount: " + iterationCount + ", maximumChange: " + maximumChange + ", iterElapsed: " + (iterEnd - iterStart) + ", memory: " + memory);
		} while (iterationCount != max_iteration); 
		//((maximumChange) >= (maximumErrorAllowed * (1.0 - DISCOUNT) / DISCOUNT) && !(maximumChange == 0));

		long endTime = System.currentTimeMillis();
		elapsedTime = endTime - startTime;
		System.out.println("elapsed time: " + elapsedTime + "ms");

		// Run garbage collector and record memory usage.
		runtime.gc();
		memory = runtime.totalMemory() - runtime.freeMemory();
		System.out.println("end memory: " + memory + "bytes");

		System.out.println("VI has converged (iteration: " + iterationCount + ")");

		// SAVE TO DISK
		env.storeOldUtility();
		String oldUtilityJson = env.toJson();

		try {
			String fileName = "./results/" + env._domain + "_d_" + env.getDiscretization() + "_min_" + String.format("%.0f", env.min_values.get(0))+ "_max_" + String.format("%.0f", env.max_values.get(0)) + ".txt";
			BufferedWriter writer = new BufferedWriter(new FileWriter(fileName));
			writer.write(oldUtilityJson);
			writer.close();
		} catch (IOException e) {
			e.printStackTrace();
		}
	}
}
