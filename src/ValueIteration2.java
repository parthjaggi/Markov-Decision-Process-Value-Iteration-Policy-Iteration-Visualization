import com.google.gson.Gson;
import java.util.*;
import java.util.Arrays;
import java.awt.Dimension;
import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;

import org.jfree.data.xy.XYSeries;
import org.jfree.data.xy.XYSeriesCollection;
import lpsolve.*;

/**
 * @author GOH KA HIAN NANYANG TECHNOLOGICAL UNIVERSITY
 *
 */
public class ValueIteration2 implements Constant {
	static DisplayGraph valueIterationGraph;
	private int iterationCount;
	// private GridWorld gw;
	private TrafficEnv tr;
	private int discretization;
	private int max_value;
	private float ratio;
	private float reward;
	// KEEP TRACK OF OLD UTILITY
	private float oldUtility[][][][][];

	// MAXIMUM CHANGE IN UTILITY
	private double maximumChange;
	// MAXIMUM ERROR ALLOWED FOR ALGORITHM, DEFAULT 0.1
	static double maximumErrorAllowed = 0.1;

	// Set the domain
	String _domain;

	// HashMap to be used in solving LP for Bandwidth problem
	public final static HashMap<Integer, double[]> _hmAction2Ub = new HashMap<Integer, double[]>();

	// Cache saving LP results
	public HashMap<int[], int[]> _hmState2NextState = new HashMap<int[], int[]>();

	// CONSTRUCTOR
	public ValueIteration2(TrafficEnv tr) {
		this.tr = tr;
	}

	// START VALUE ITERATION
	public void startIteration() {
		iterationCount = 0;
		discretization = tr.getDiscretization();
		max_value = tr.getMaxValue();
		ratio = tr.getRatio();

		// INITIALIZE AUXILLARY ARRAY
		this.oldUtility = new float[discretization][discretization][discretization][discretization][discretization];
		// UPDATE STATES HAVING NEIGHBOR WALLS
		// gw.updateIsWall();

		// FOR GRAPH PLOTTING
		// final XYSeriesCollection collection = new XYSeriesCollection();
		;
		// final XYSeries series[][] = new XYSeries[gw.getCols()][gw.getRows()];
		// // INSTANTIATE SERIES AND INITIAL UTILITY PLOTTING (0,0)
		// for (int i = 0; i < gw.getCols(); i++)
		// 	for (int j = 0; j < gw.getRows(); j++) {
		// 		series[i][j] = new XYSeries("(" + i + "," + j + ")");
		// 		series[i][j].add(0, 0);
		// 	}

		// INITIAL STEP: SET U(S) = 0
		setUtlityZero();

		long startTime = System.currentTimeMillis();

		// ITERATE UNTIL CONVERGENCE BASED ON maximumErrorAllowed
		do {
			iterationCount++;
			// MAXIMUM CHANGES IN UTILITY
			maximumChange = 0;
			// KEEP TRACK OLD UTILITY
			storeOldUtility();
			// UPDATE NEW UTILITY VALUE
			updateUtility();

			// PLOT GRAPH AND CALCULATE MAXIMUM CHANGES IN UTILITY
			double differences = 0;
			for (int i1 = 0; i1 < discretization; i1++)
				for (int i2 = 0; i2 < discretization; i2++)
					for (int i3 = 0; i3 < discretization; i3++)
						for (int i4 = 0; i4 < discretization; i4++)
							for (int i5 = 0; i5 < discretization; i5++) {
								// series[i][j].add(iterationCount, gw.states[i][j].getUtility());
								differences = Math.abs(tr.states[i1][i2][i3][i4][i5].getUtility() - oldUtility[i1][i2][i3][i4][i5]);
								if (differences > maximumChange)
									maximumChange = differences;
							}
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

		// SAVE TO DISK
		storeOldUtility();
		Gson gson = new Gson();
		String oldUtilityJson = gson.toJson(oldUtility);

		try {
			String fileName = "./results/" + discretization + ".txt";
			BufferedWriter writer = new BufferedWriter(new FileWriter(fileName));
			writer.write(oldUtilityJson);
			writer.close();
		} catch (IOException e) {
			e.printStackTrace();
		}

		// DISPLAY OPTIMAL POLICY
		// gw.displayOptimalPolicy();

		// COMBINE ALL XYSERIES
		// for (int i = 0; i < gw.getCols(); i++)
		// 	for (int j = 0; j < gw.getRows(); j++) {
		// 		if (gw.states[i][j].isWall())
		// 			continue;
		// 		else
		// 			collection.addSeries(series[i][j]);
		// 	}
		// /* DISPLAY GRAPH AND STATISTIC */
		// Main.numIteration.setText("Total Iteration Count: " + iterationCount);
		// Main.numStates.setText("Numbers of states: " + collection.getSeriesCount());
		// valueIterationGraph = new DisplayGraph("Value Iteration (Max error: " + maximumErrorAllowed + ")", collection);
		// valueIterationGraph.setSize(new Dimension(720, 720));
		// valueIterationGraph.setLocationRelativeTo(null);
		// valueIterationGraph.setVisible(true);
	}

	// SET UTILITY OF ALL STATES TO 0
	public void setUtlityZero() {
		// FOREACH STATE UPDATE THE UTILITY
		for (int i1 = 0; i1 < discretization; i1++)
			for (int i2 = 0; i2 < discretization; i2++)
				for (int i3 = 0; i3 < discretization; i3++)
					for (int i4 = 0; i4 < discretization; i4++) 
						for (int i5 = 0; i5 < discretization; i5++) {
							tr.states[i1][i2][i3][i4][i5].setUtility(0);
						}
	}

	// STORE OLD UTILITY VALUES
	public void storeOldUtility() {
		// FOREACH STATE UPDATE THE UTILITY
		for (int i1 = 0; i1 < discretization; i1++)
			for (int i2 = 0; i2 < discretization; i2++)
				for (int i3 = 0; i3 < discretization; i3++)
					for (int i4 = 0; i4 < discretization; i4++) 
						for (int i5 = 0; i5 < discretization; i5++) {
							oldUtility[i1][i2][i3][i4][i5] = tr.states[i1][i2][i3][i4][i5].getUtility();
						}
	}

	public int[] getNextState(int i1, int i2, int i3, int i4, int i5, int action) {
		int j1 = i1, j2 = i2, j3 = i3;
		// float reward = 0;

		if (action == WE) {
			// solve LP
			int[] next_states = new int[3];
			next_states = optimizedTransition(new int[] {i1, i2, i3}, new int[0], action, "traffic");
			// i4, i5 remain the same.
			j1 = next_states[0];
			j2 = next_states[1];
			j3 = next_states[2];
			reward = (i1 - j1) * ratio;
		} else {
			// i1, i2, i3 remain the same.
			int idxToTransfer = Math.min(Math.min(Math.max(Math.round(i4), 0), Math.round(discretization - i5 - 1)), (int) (15 / ratio)); 
			i4 = Math.round(i4 - idxToTransfer);
			i5 = Math.round(i5 + idxToTransfer);
			reward = idxToTransfer * ratio;
		}
		return new int[] { j1, j2, j3, i4, i5 };
		// return new Object[] { j1, j2, j3, i4, i5, reward };
	}


	public float getStateFromIndex(int i){
		return (float)(i * ratio);
	}

	public int getIndexFromState(float q){
		int index = (int)Math.round(q / ratio);
		if (index >= discretization){
			index = discretization - 1;
		}
		return index;
	}

	/* 
	TODO: when returning the next states, need to consider feasibility with respect to discretization. 
	*/
	public int[] optimizedTransition(int[] real_states, int[] binary_states, int action, String domain){
		_domain = domain.intern();

		if (_domain.equalsIgnoreCase("traffic")){
			// real_states = (q1, ..., q5), binary_states = (). We need q2, q3 to solve for dq2 and dq3
			// When (action == 0) (East-West), need to solve an LP; 
			// whereas for (action == 1), we don't need to solve.. (should be handled in other parts of the code)
			return optimizedTransitionTraffic(real_states, action);
		}
		// else if(_domain.equalsIgnoreCase("reservoir")){
		// 	// real_states = (l1, l2), binary_states = (r). We need (l1, l2, r, action) to determine the next state.
		// 	return optimizedTransitionReservoir(real_states, binary_states[0], action);
		// }
		// else if(_domain.equalsIgnoreCase("bandwidth")){
		// 	// real_states = (d), binary_states = (l). action can be an integer from 0 to 6.
		// 	// The LP does not depend on current state, but the action will determine which links of the graph will be on.
		// 	return optimizedTransitionBandwidth(real_states[0], binary_states[0], action);
		// }
		else{
			System.out.println("Warning: Unrecognized domain is provided!!");
			System.exit(1);
		}
		return null;
	}

	public int[] optimizedTransitionTraffic(int[] states, int action){
		float q1, q2, q3;
		float dq2, dq3;
		int[] next_states = new int[3];
		int[] _tempStates = new int[3];
		
		if (action == 1){
			System.out.println("Warning: a=1 case should have been handled in different parts than here!");
			System.exit(1);
		}
		// Check cache
		_tempStates = _hmState2NextState.get(states);
		if (_tempStates != null){
			return _tempStates;
		}

		q1 = getStateFromIndex(states[0]);
		q2 = getStateFromIndex(states[1]);
		q3 = getStateFromIndex(states[2]);

		try {
			// An LP solver object initialized with 2 variables (no constraints)
			LpSolve solver = LpSolve.makeLp(0, 2);	
			
			// Verbose level
			solver.setVerbose(LpSolve.CRITICAL);

			// Add constraints
			solver.strAddConstraint("1 1", LpSolve.LE, 20);
			solver.strAddConstraint("1 0", LpSolve.LE, 120-q2);
			solver.strAddConstraint("0 1", LpSolve.LE, 100-q3);

			// Bound constraints
			solver.setLowbo(1, 0);			// dq2 >= 0
			solver.setLowbo(2, 0);			// dq3 >= 0

			// Set objective function
			solver.strSetObjFn("1 1");	// obj = dq2 + dq3

			// Need to maximize
			solver.setMaxim();

			// Solve the problem
			solver.solve();

			// Retrieve solution
			double[] var = solver.getPtrVariables();

			dq2 = (float)var[0];
			dq3 = (float)var[1];
			
			q2 += dq2;
			q3 += dq3;
			q1 -= (dq2 + dq3);
			
			next_states[0] = getIndexFromState(q1);					// TODO: are these values valid with the current discretization level?
			next_states[1] = getIndexFromState(q2);
			next_states[2] = getIndexFromState(q3);

			// delte the problem and free memory
			solver.deleteLp();

			// Save the result in cache
			_hmState2NextState.put(states, next_states);

			return next_states;

		}
		catch (LpSolveException e) {
			e.printStackTrace();
		}
		return null;
	}

	public double[] optimizedTransitionReservoir(double[] levels, int rain, int action){
		// real_states = (l1, l2), binary_states = (r). 
		// We need (l1, l2, r, action) to determine the next state.
		// action = 0: block the water flow, i.e., q1 = 0. 
		// action = 1: q1 >= 0
		double l1, l2;
		double q1, q2;
		
		try {
			// An LP solver object initialized with 2 variables (no constraints)
			LpSolve solver = LpSolve.makeLp(0, 2);	
			
			// Add constraints
			double rhs1, rhs2, rhs3, rhs4;
			rhs1 = 0.98 * levels[0] + 200 * rain - 1000;
			rhs2 = 0.98 * levels[0] + 200 * rain - 3000;
			rhs3 = 700 - 0.98 * levels[1] - 200 * rain;
			rhs4 = 1500 - 0.98 * levels[1] - 200 * rain;
			solver.strAddConstraint("1 0", LpSolve.LE, rhs1);
			solver.strAddConstraint("1 0", LpSolve.GE, rhs2);
			solver.strAddConstraint("1 -1", LpSolve.GE, rhs3);
			solver.strAddConstraint("1 -1", LpSolve.LE, rhs4);

			// Bound constraints: 0 <= q1 <= 250 * action; 0 <= q2 <= 300
			solver.setLowbo(0, 0);
			solver.setUpbo(0, 250 * action);
			solver.setLowbo(1, 0);
			solver.setUpbo(1, 300);
			
			// Set objective function
			solver.strSetObjFn("0 1");		// obj = q2

			// Solve the LP
			solver.solve();

			// Retrieve solution
			double[] var = solver.getPtrVariables();

			q1 = var[0];
			q2 = var[1];

			// Compute the state transition and return the next state
			double[] next_levels = new double[2];
			next_levels[0] = 0.98 * levels[0] - q1 + 200 * rain;			// TODO: are these values valid with the current discretization level?
			next_levels[1] = 0.98 * levels[1] + q1 - q2 + 200 * rain;
			return next_levels;
		}
		catch (LpSolveException e) {
			e.printStackTrace();
		}
		return null;
	}

	public double[] optimizedTransitionBandwidth(double demand, int level, int action){
		double x_o1, x_o2, x_12, x_1e, x_2e;
		double ub;
		double next_demand, dnew;

		double[] upper_bounds = _hmAction2Ub.get(action);

		try {
			// An LP solver object initialized with 5 variables (no constraints)
			LpSolve solver = LpSolve.makeLp(0, 5);

			// Add constraints
			solver.strAddConstraint("1 0 -1 -1 0", LpSolve.EQ, 0);
			solver.strAddConstraint("0 -1 -1 0 1", LpSolve.EQ, 0);

			
			// Bound constraints
			for (int i=0; i<5; i++){
				ub = upper_bounds[i];
				solver.setLowbo(i, 0);
				solver.setUpbo(i, ub);
			}
			
			// Set objective function
			solver.strSetObjFn("1 1");		// obj = xo1 + xo2

			// Solve the LP
			solver.solve();

			// Retrieve solution
			double max_flow = solver.getObjective();

			// Compute the state transition and return the next state
			dnew = (action == 0)? 1200: 2500;
			if (demand > max_flow){
				next_demand = demand - max_flow + dnew;				// TODO: are these values valid with the current discretization level?
			} else {
				next_demand = dnew;
			}
			return new double[] {next_demand};
		}
		catch (LpSolveException e){
			e.printStackTrace();
		}
		return null;
	}

	private void updateHmAction2Ub(){
		double[] capacity = {2100, 1800, 1000, 1500, 1700};
		double[][] edgesInAction = {{1, 0, 0, 1, 0}, {1, 0, 1, 0, 1}, 
									{0, 1, 0, 0, 1}, {1, 0, 1, 1, 1}, 
									{1, 1, 0, 1, 1}, {1, 1, 1, 0, 1}, {1, 1, 1, 1, 1}};
		double[] upper_bounds =  new double[5];

		for (int a=0; a<7; a++){
			for (int i=0; i<5; i++){
				upper_bounds[i] = capacity[i] * edgesInAction[a][i];
			}
			_hmAction2Ub.put(a, upper_bounds);
		}
	}

	// U(s) = R(s) + discount*MAX(expected utility of an action)
	public void updateUtility() {
		float actionUtility[] = new float[2];
		// double rewards[] = new double[2];
		int[] nextState;
		int bestAction;

		// FOREACH STATE UPDATE THE UTILITY
		for (int i1 = 0; i1 < discretization; i1++)
			for (int i2 = 0; i2 < discretization; i2++)
				for (int i3 = 0; i3 < discretization; i3++)
					for (int i4 = 0; i4 < discretization; i4++) 
						for (int i5 = 0; i5 < discretization; i5++) {

							// handle the case when q1 (i1 * ratio) is < 20.
							if (i1 * ratio < 20)
								continue;

							Arrays.fill(actionUtility, 0);
							// Arrays.fill(rewards, 0);
							
							// ACTION: WE
							nextState = getNextState(i1, i2, i3, i4, i5, WE);
							actionUtility[WE] = reward + ((float) DISCOUNT) * oldUtility[nextState[0]][nextState[1]][nextState[2]][nextState[3]][nextState[4]];
							// rewards[WE] = reward;

							// ACTION: NS
							nextState = getNextState(i1, i2, i3, i4, i5, NS);
							actionUtility[NS] = reward + ((float) DISCOUNT) * oldUtility[nextState[0]][nextState[1]][nextState[2]][nextState[3]][nextState[4]];
							// rewards[NS] = reward;

							// SET THE ACTION WITH HIGHEST EXPECTED UTILITY
							bestAction = actionUtility[WE] > actionUtility[NS] ? WE : NS;
							tr.states[i1][i2][i3][i4][i5].setBestAction(bestAction);

							// UPDATE UTILITY BASED ON BELLMAN EQUATION
							// tr.states[i1][i2][i3][i4][i5].setUtility(rewards[bestAction] + DISCOUNT * actionUtility[bestAction]);
							tr.states[i1][i2][i3][i4][i5].setUtility(actionUtility[bestAction]);
			}
	}

	// //NOT IN USE
	// public boolean convergence(){
	// for (int i = 0; i < gw.getRows(); i++)
	// for (int j = 0; j < gw.getCols(); j++)
	// if(oldUtility[i][j] != gw.states[i][j].getUtility())
	// return false;
	// System.out.println("num of iteration :"+ iterationCount);
	// return true;
	// }

}
