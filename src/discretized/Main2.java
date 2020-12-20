package discretized;
import java.util.*;
import java.text.DecimalFormat;
import discretized.Domain.TrafficEnv;
import discretized.Domain.ReservoirEnv;
import discretized.Domain.BandwidthEnv;
import discretized.Domain.Environment;
/**
 * @author GOH KA HIAN NANYANG TECHNOLOGICAL UNIVERSITY
 *
 */
final public class Main2 {
	// GRIDWORLD AND ALGORITHM
	// static GridWorld gw;
	// static ValueIteration valueIterate;
	// static TrafficEnv tr;
	static Domain _domain;
	public static int max_value;
	public static int min_value;
	static ValueIteration2 valueIterate;
	static boolean valueRunning = false;
	public static Environment _env;
	// MAIN
	public static void main(String[] args){
		String domain = args[0];
		min_value = Integer.parseInt(args[1]);
		max_value = Integer.parseInt(args[2]); 
		_domain = new Domain(domain);

		for (int d = 4; d <= 20; d++) {
			System.out.println("");
			System.out.println("=========================================================");
			System.out.println("d: " + d);

			if (domain.equalsIgnoreCase("traffic")){
				_env = new TrafficEnv(/*min_value*/ min_value, /*max_value*/ max_value, /*discretization*/ d);
			}
			else if (domain.equalsIgnoreCase("reservoir")){
				_env = new ReservoirEnv(/*min_value*/ min_value, /*max_value*/ max_value, d);
			}
			else if (domain.equalsIgnoreCase("bandwidth")){
				_env = new BandwidthEnv(/*min_value*/ min_value, /*max_value*/ max_value, d);
			}
			else{
				System.exit(1);
			}

			// tr = new TrafficEnv(100, d);
			valueIterate = new ValueIteration2(((Environment)_env), domain);
			valueIterate.startIteration();
		}
		// policyIterate = new PolicyIteration(gw);
		System.out.println("Running discretized MDP with optimized transitions, Author: Parth Jaggi, Jihwan Jeong");
		// initializeGui();

	//	uncomment for 100x100 GridWorld
	//	gw.setColsRowsSize(100);
	//	gw.stateSizeChange();
	}
}
