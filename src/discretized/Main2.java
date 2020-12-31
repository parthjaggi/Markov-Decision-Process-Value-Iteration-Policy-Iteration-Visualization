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
	public static HashMap<Integer, long[]> spaceTime = new HashMap<Integer, long[]>();
	public static HashMap<Integer, long[][]> iterSpaceTime = new HashMap<Integer, long[][]>();
	// MAIN
	public static void main(String[] args){
		String domain = args[0];
		int num_iteration = Integer.parseInt(args[1]);
		_domain = new Domain(domain);

		int startD = 0, endD = 0, stepD = 0;
		if (domain.equalsIgnoreCase("traffic")) {
			startD = 4; endD = 14; stepD = 2;
		}
		else if (domain.equalsIgnoreCase("reservoir")) {
			startD = 50; endD = 1000; stepD = 50;
		}
		else if (domain.equalsIgnoreCase("bandwidth")) {
			startD = 2; endD = 16; stepD = 2;
		}
		else {
			System.exit(1);
		}

		for (int d = startD; d <= endD; d = d + stepD) {
			System.out.println("");
			System.out.println("=========================================================");
			System.out.println("d: " + d);

			if (domain.equalsIgnoreCase("traffic")){
				_env = new TrafficEnv(/*discretization*/ d);
			}
			else if (domain.equalsIgnoreCase("reservoir")){
				_env = new ReservoirEnv(d);
			}
			else if (domain.equalsIgnoreCase("bandwidth")){
				_env = new BandwidthEnv(d);
			}
			else{
				System.exit(1);
			}

			// Set min, max values and ratio
			_env.setMinMaxValues(Arrays.copyOfRange(args, 2, args.length), d);
			_env.setEnvName();

			// tr = new TrafficEnv(100, d);
			valueIterate = new ValueIteration2(((Environment)_env), domain);
			valueIterate.startIteration(num_iteration);

			spaceTime.put(d, new long[]{valueIterate.elapsedTime, valueIterate.memory});
			iterSpaceTime.put(d, new long[][]{valueIterate.iterationElapsedTime, valueIterate.iterationMemory});
		}

		String spaceTimeFile = "./results/" + domain + "_space_time.txt";
		Utils.saveToFile(spaceTime, spaceTimeFile);
		String iterSpaceTimeFile = "./results/" + domain + "_iter_space_time.txt";
		Utils.saveToFile(iterSpaceTime, iterSpaceTimeFile);
		System.out.println("Running discretized MDP with optimized transitions, Author: Parth Jaggi, Jihwan Jeong");
	}
}
