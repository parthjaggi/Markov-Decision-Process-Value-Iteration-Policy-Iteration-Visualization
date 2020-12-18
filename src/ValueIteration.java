import java.util.*;
import java.util.Arrays;
import java.awt.Dimension;
import org.jfree.data.xy.XYSeries;
import org.jfree.data.xy.XYSeriesCollection;
import lpsolve.*;

/**
 * @author GOH KA HIAN NANYANG TECHNOLOGICAL UNIVERSITY
 *
 */
public class ValueIteration implements Constant {
	static DisplayGraph valueIterationGraph;
	private int iterationCount;
	private GridWorld gw;
	// KEEP TRACK OF OLD UTILITY
	private double oldUtility[][];

	// MAXIMUM CHANGE IN UTILITY
	private double maximumChange;
	// MAXIMUM ERROR ALLOWED FOR ALGORITHM, DEFAULT 0.1
	static double maximumErrorAllowed = 0.1;
	
	// Set the domain
	String _domain;

	// HashMap to be used in solving LP for Bandwidth problem
	public final static HashMap<Integer, double[]> _hmAction2Ub = new HashMap<Integer, double[]>();

	// CONSTRUCTOR
	public ValueIteration(GridWorld gw) {
		this.gw = gw;
	}

	// START VALUE ITERATION
	public void startIteration() {
		iterationCount = 0;

		// INITIALIZE AUXILLARY ARRAY
		this.oldUtility = new double[gw.getRows()][gw.getCols()];

		// Initialize action - edge mapping for the bandwidth problem
		updateHmAction2Ub();

		// UPDATE STATES HAVING NEIGHBOR WALLS
		gw.updateIsWall();

		// FOR GRAPH PLOTTING
		final XYSeriesCollection collection = new XYSeriesCollection();
		;
		final XYSeries series[][] = new XYSeries[gw.getCols()][gw.getRows()];
		// INSTANTIATE SERIES AND INITIAL UTILITY PLOTTING (0,0)
		for (int i = 0; i < gw.getCols(); i++)
			for (int j = 0; j < gw.getRows(); j++) {
				series[i][j] = new XYSeries("(" + i + "," + j + ")");
				series[i][j].add(0, 0);
			}

		// INITIAL STEP: SET U(S) = 0
		setUtlityZero();

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
			for (int i = 0; i < gw.getCols(); i++)
				for (int j = 0; j < gw.getRows(); j++) {
					series[i][j].add(iterationCount, gw.states[i][j].getUtility());
					differences = Math.abs(gw.states[i][j].getUtility() - oldUtility[i][j]);
					if (differences > maximumChange)
						maximumChange = differences;
				}
		} while ((maximumChange) >= (maximumErrorAllowed * (1.0 - DISCOUNT) / DISCOUNT) && !(maximumChange == 0));

		// DISPLAY OPTIMAL POLICY
		gw.displayOptimalPolicy();

		// COMBINE ALL XYSERIES
		for (int i = 0; i < gw.getCols(); i++)
			for (int j = 0; j < gw.getRows(); j++) {
				if (gw.states[i][j].isWall())
					continue;
				else
					collection.addSeries(series[i][j]);
			}
		/* DISPLAY GRAPH AND STATISTIC */
		Main.numIteration.setText("Total Iteration Count: " + iterationCount);
		Main.numStates.setText("Numbers of states: " + collection.getSeriesCount());
		valueIterationGraph = new DisplayGraph("Value Iteration (Max error: " + maximumErrorAllowed + ")", collection);
		valueIterationGraph.setSize(new Dimension(720, 720));
		valueIterationGraph.setLocationRelativeTo(null);
		valueIterationGraph.setVisible(true);
	}

	// SET UTILITY OF ALL STATES TO 0
	public void setUtlityZero() {
		for (int i = 0; i < gw.getCols(); i++)
			for (int j = 0; j < gw.getRows(); j++)
				gw.states[i][j].setUtility(0);
	}

	// STORE OLD UTILITY VALUES
	public void storeOldUtility() {
		for (int i = 0; i < gw.getCols(); i++)
			for (int j = 0; j < gw.getRows(); j++) {
				// IGNORE WALL
				if (gw.states[i][j].isWall())
					continue;
				oldUtility[i][j] = gw.states[i][j].getUtility();
			}
	}

	// U(s) = R(s) + discount*MAX(expected utility of an action)
	public void updateUtility() {
		double actionUtility[] = new double[4];
		// FOREACH STATE UPDATE THE UTILITY
		for (int i = 0; i < gw.getCols(); i++)
			for (int j = 0; j < gw.getRows(); j++) {
				// IGNORE UPDATING WALL
				if (gw.states[i][j].isWall())
					continue;
				// RESET AUXILIARY ARRAY
				Arrays.fill(actionUtility, 0);

				// if north of state has wall
				if (gw.states[i][j].isNorthWall()) {
					actionUtility[UP] += FRONT_CHANCE * oldUtility[i][j];
					actionUtility[LEFT] += RIGHT_CHANCE * oldUtility[i][j];
					actionUtility[RIGHT] += LEFT_CHANCE * oldUtility[i][j];
				} else {// no wall on north
					actionUtility[UP] += FRONT_CHANCE * oldUtility[i][j - 1];
					actionUtility[LEFT] += RIGHT_CHANCE * oldUtility[i][j - 1];
					actionUtility[RIGHT] += LEFT_CHANCE * oldUtility[i][j - 1];
				}
				// if south has wall
				if (gw.states[i][j].isSouthWall()) {
					actionUtility[DOWN] += FRONT_CHANCE * oldUtility[i][j];
					actionUtility[LEFT] += LEFT_CHANCE * oldUtility[i][j];
					actionUtility[RIGHT] += RIGHT_CHANCE * oldUtility[i][j];
				} else {// no wall on south
					actionUtility[DOWN] += FRONT_CHANCE * oldUtility[i][j + 1];
					actionUtility[LEFT] += LEFT_CHANCE * oldUtility[i][j + 1];
					actionUtility[RIGHT] += RIGHT_CHANCE * oldUtility[i][j + 1];
				}
				// if west has wall
				if (gw.states[i][j].isWestWall()) {
					actionUtility[DOWN] += RIGHT_CHANCE * oldUtility[i][j];
					actionUtility[LEFT] += FRONT_CHANCE * oldUtility[i][j];
					actionUtility[UP] += LEFT_CHANCE * oldUtility[i][j];
				} else {// no wall on west
					actionUtility[DOWN] += RIGHT_CHANCE * oldUtility[i - 1][j];
					actionUtility[LEFT] += FRONT_CHANCE * oldUtility[i - 1][j];
					actionUtility[UP] += LEFT_CHANCE * oldUtility[i - 1][j];
				}
				// if east has wall
				if (gw.states[i][j].isEastWall()) {
					actionUtility[DOWN] += LEFT_CHANCE * oldUtility[i][j];
					actionUtility[RIGHT] += FRONT_CHANCE * oldUtility[i][j];
					actionUtility[UP] += RIGHT_CHANCE * oldUtility[i][j];
				} else {// no wall on east
					actionUtility[DOWN] += LEFT_CHANCE * oldUtility[i + 1][j];
					actionUtility[RIGHT] += FRONT_CHANCE * oldUtility[i + 1][j];
					actionUtility[UP] += RIGHT_CHANCE * oldUtility[i + 1][j];
				}

				// SET THE ACTION WITH HIGHEST EXPECTED UTILITY
				gw.states[i][j].setBestAction(DOWN);
				if (actionUtility[UP] > actionUtility[gw.states[i][j].getBestAction()])
					gw.states[i][j].setBestAction(UP);
				if (actionUtility[LEFT] > actionUtility[gw.states[i][j].getBestAction()])
					gw.states[i][j].setBestAction(LEFT);
				if (actionUtility[RIGHT] > actionUtility[gw.states[i][j].getBestAction()])
					gw.states[i][j].setBestAction(RIGHT);

				// UPDATE UTILITY BASED ON BELLMAN EQUATION
				gw.states[i][j].setUtility(
						gw.states[i][j].getReward() + DISCOUNT * actionUtility[gw.states[i][j].getBestAction()]);
				// System.out.println("s("+i+","+j+") :" +
				// gw.states[i][j].getUtility());
			}
	}

	/* 
	TODO: when returning the next states, need to consider feasibility with respect to discretization. 
	*/
	public double[] optimizedTransition(double[] real_states, int[] binary_states, int action, String domain){
		_domain = domain.intern();

		if (_domain.equalsIgnoreCase("traffic")){
			// real_states = (q1, ..., q5), binary_states = (). We need q2, q3 to solve for dq2 and dq3
			// When (action == 0) (East-West), need to solve an LP; 
			// whereas for (action == 1), we don't need to solve.. (should be handled in other parts of the code)
			return optimizedTransitionTraffic(real_states, action);
		}
		else if(_domain.equalsIgnoreCase("reservoir")){
			// real_states = (l1, l2), binary_states = (r). We need (l1, l2, r, action) to determine the next state.
			return optimizedTransitionReservoir(real_states, binary_states[0], action);
		}
		else if(_domain.equalsIgnoreCase("bandwidth")){
			// real_states = (d), binary_states = (l). action can be an integer from 0 to 6.
			// The LP does not depend on current state, but the action will determine which links of the graph will be on.
			return optimizedTransitionBandwidth(real_states[0], binary_states[0], action);
		}
		else{
			System.out.println("Warning: Unrecognized domain is provided!!");
			System.exit(1);
		}
		return null;
	}

	public double[] optimizedTransitionTraffic(double[] states, int action){
		double q1, q2, q3;
		double dq2, dq3;
		if (action == 1){
			System.out.println("Warning: a=1 case should have been handled in different parts than here!");
			System.exit(1);
		}
		q1 = states[0];
		q2 = states[1];
		q3 = states[2];
		
		try {
			// An LP solver object initialized with 2 variables (no constraints)
			LpSolve solver = LpSolve.makeLp(0, 2);	
					
			// Add constraints
			solver.strAddConstraint("1 1", LpSolve.LE, 20);
			solver.strAddConstraint("1 0", LpSolve.LE, 120-q2);
			solver.strAddConstraint("0 1", LpSolve.LE, 100-q3);

			// Bound constraints
			solver.setLowbo(0, 0);			// dq2 >= 0
			solver.setLowbo(1, 0);			// dq3 >= 0

			// Set objective function
			solver.strSetObjFn("1 1");	// obj = dq2 + dq3

			// Solve the problem
			solver.solve();

			// Retrieve solution
			double[] var = solver.getPtrVariables();

			dq2 = var[0];
			dq3 = var[1];
			
			q2 += dq2;
			q3 += dq3;
			q1 -= (dq2 + dq3);
			
			double[] next_states = states;
			next_states[0] = q1;					// TODO: are these values valid with the current discretization level?
			next_states[1] = q2;
			next_states[2] = q3;

			// delte the problem and free memory
			solver.deleteLp();

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
