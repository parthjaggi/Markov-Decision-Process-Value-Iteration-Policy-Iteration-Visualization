package discretized;
import java.util.*;
import java.util.Arrays;
import java.awt.Dimension;
import org.jfree.data.xy.XYSeries;
import org.jfree.data.xy.XYSeriesCollection;
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
	// private GridWorld gw;
	public Environment env;
	private int discretization;
	private int max_value;
	private float ratio;
	private float reward;
	// KEEP TRACK OF OLD UTILITY
	// private float oldUtility[][][][][];

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
	}

	// START VALUE ITERATION
	public void startIteration() {
		iterationCount = 0;
		discretization = env.getDiscretization();
		max_value = env.getMaxValue();
		ratio = env.getRatio();

		// INITIALIZE AUXILLARY ARRAY
		// this.oldUtility = new float[discretization][discretization][discretization][discretization][discretization];
		
		// INITIAL STEP: SET U(S) = 0
		// setUtlityZero();
		env.setUtilityZero();

		long startTime = System.currentTimeMillis();

		// ITERATE UNTIL CONVERGENCE BASED ON maximumErrorAllowed
		do {
			iterationCount++;
			// MAXIMUM CHANGES IN UTILITY
			maximumChange = 0;
			// KEEP TRACK OLD UTILITY
			env.storeOldUtility();
			
			// UPDATE NEW UTILITY VALUE
			env.updateUtility();

			// CALCULATE MAXIMUM CHANGES IN UTILITY
			maximumChange = env.computeMaximumDifference();
		} while ((maximumChange) >= (maximumErrorAllowed * (1.0 - DISCOUNT) / DISCOUNT) && !(maximumChange == 0));

		long endTime = System.currentTimeMillis();
		long elapsedTime = endTime - startTime;
		System.out.println("elapsed time: " + elapsedTime + "ms");

		// Get the Java runtime and Run the garbage collector
		Runtime runtime = Runtime.getRuntime();
		runtime.gc();
		long memory = runtime.totalMemory() - runtime.freeMemory();
		System.out.println("end memory: " + memory + "bytes");

		System.out.println("VI has converged");
	}
}
