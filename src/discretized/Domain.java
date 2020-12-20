package discretized;
import java.lang.Math;
import lpsolve.*;
import java.util.*;

/**
 * @author Humpty Dumpty.
 *
 */
public class Domain implements Constant {
	/**
	 * 
	 */
	public static String _str_domain;

	public Domain(String domain) {
		_str_domain = domain.intern();
	}

	public void resetEnvironment(int discretization){
	// reset the environment given discretization

	} 

	public static abstract class Environment{
		public static int discretization;
		public static int max_value;
		public static int num_real_states;
		public static int num_binary_states;
		public static int dim_binary_states;
		public static int dim_real_states;
		public static float ratio;
		public static String _domain;
		public static float reward;

		// Cache saving LP results
		public static HashMap<int[], int[]> _hmState2NextState = new HashMap<int[], int[]>();

		// public Environment(int max_value, int discretization){
		// 	max_value = max_value;
		// 	discretization = discretization;
		// 	ratio = (float) max_value / discretization;
		// 	this.modifyStateSize();
		// }

		public abstract void modifyStateSize();
		protected abstract void initiate();

		public abstract void storeOldUtility();
		public abstract void setUtilityZero();
		public abstract void updateUtility();
		public abstract double computeMaximumDifference();
		public abstract int[] optimizedTransition(int[] current_state, int action);
		public int getDiscretization() {
			return discretization;
		}
	
		public int getMaxValue() {
			return max_value;
		}
	
		public float getRatio() {
			return ratio;
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

		public int[] getNextState(int[] current_state, int action){
			int[] next_state = optimizedTransition(current_state, action);
			return next_state;
		}
	}

	public static class TrafficEnv extends Environment{
		// Some variable declaration
		DState[][][][][] states;
		public float oldUtility[][][][][];

		// Constructor
		public TrafficEnv(int max_value, int discretization){
			// super(max_value, discretization);
			super.max_value = max_value;
			super.discretization = discretization;
			super.ratio = (float) max_value / discretization;
			this.modifyStateSize();
		}

		protected void initiate(){
			for (int i1 = 0; i1 < discretization; i1++)
				for (int i2 = 0; i2 < discretization; i2++)
					for (int i3 = 0; i3 < discretization; i3++)
						for (int i4 = 0; i4 < discretization; i4++) 
							for (int i5 = 0; i5 < discretization; i5++) {
								states[i1][i2][i3][i4][i5] = new DState(this);
							}
			oldUtility = new float[discretization][discretization][discretization][discretization][discretization];
		}

		public void modifyStateSize(){
			states = new DState[discretization][discretization][discretization][discretization][discretization];
			initiate();
		}

		public void updateUtility(){
			float actionUtility[] = new float[2];
			int[] nextState;
			int bestAction;

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
								nextState = getNextState(new int[]{i1, i2, i3, i4, i5}, WE);
								actionUtility[WE] = reward + ((float) DISCOUNT) * oldUtility[nextState[0]][nextState[1]][nextState[2]][nextState[3]][nextState[4]];
								// rewards[WE] = reward;

								// ACTION: NS
								nextState = getNextState(new int[]{i1, i2, i3, i4, i5}, NS);
								actionUtility[NS] = reward + ((float) DISCOUNT) * oldUtility[nextState[0]][nextState[1]][nextState[2]][nextState[3]][nextState[4]];
								// rewards[NS] = reward;

								// SET THE ACTION WITH HIGHEST EXPECTED UTILITY
								bestAction = actionUtility[WE] > actionUtility[NS] ? WE : NS;
								states[i1][i2][i3][i4][i5].setBestAction(bestAction);

								// UPDATE UTILITY BASED ON BELLMAN EQUATION
								// tr.states[i1][i2][i3][i4][i5].setUtility(rewards[bestAction] + DISCOUNT * actionUtility[bestAction]);
								states[i1][i2][i3][i4][i5].setUtility(actionUtility[bestAction]);			
							}
		}

		public int[] optimizedTransition(int[] current_state, int action){
			float q1, q2, q3;
			float dq2, dq3;
			int[] next_state = new int[5];
			int[] _tempStates;
			
			if (action == 1){
				int idxToTransfer = Math.min(Math.min(Math.max(Math.round(current_state[3]), 0), Math.round(discretization - current_state[4] - 1)), (int) (15 / ratio)); 	
				next_state[0] = current_state[0];		// q1, q2, q3 remain the same
				next_state[1] = current_state[1];
				next_state[2] = current_state[2];
				next_state[3] = Math.round(current_state[3] - idxToTransfer);
				next_state[4] = Math.round(current_state[4] + idxToTransfer);
				reward = idxToTransfer * ratio;
				return next_state;
			}
	
			_tempStates = new int[] {current_state[0], current_state[1], current_state[2]};
	
			// Check cache
			_tempStates = _hmState2NextState.get(_tempStates);
			if (_tempStates != null){
				return _tempStates;
			}
	
			q1 = getStateFromIndex(current_state[0]);
			q2 = getStateFromIndex(current_state[1]);
			q3 = getStateFromIndex(current_state[2]);
	
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
				
				next_state[0] = getIndexFromState(q1);					// TODO: are these values valid with the current discretization level?
				next_state[1] = getIndexFromState(q2);
				next_state[2] = getIndexFromState(q3);
				next_state[3] = current_state[3];					// q4, q5 remain the same
				next_state[4] = current_state[4];
				// delte the problem and free memory
				solver.deleteLp();
	
				// Save the result in cache
				_hmState2NextState.put(current_state, next_state);
	
				// Compute reward
				reward = (current_state[0] - next_state[0]) * ratio;
				return next_state;
			}
			catch (LpSolveException e) {
				e.printStackTrace();
			}
			return null;
		}

		public void storeOldUtility(){
			for (int i1 = 0; i1 < discretization; i1++)
				for (int i2 = 0; i2 < discretization; i2++)
					for (int i3 = 0; i3 < discretization; i3++)
						for (int i4 = 0; i4 < discretization; i4++) 
							for (int i5 = 0; i5 < discretization; i5++) {
								oldUtility[i1][i2][i3][i4][i5] = states[i1][i2][i3][i4][i5].getUtility();
							}
		}

		public void setUtilityZero(){
			for (int i1 = 0; i1 < discretization; i1++)
				for (int i2 = 0; i2 < discretization; i2++)
					for (int i3 = 0; i3 < discretization; i3++)
						for (int i4 = 0; i4 < discretization; i4++) 
							for (int i5 = 0; i5 < discretization; i5++) {
								states[i1][i2][i3][i4][i5].setUtility(0);
							}
		}
		public double computeMaximumDifference(){
			double differences = 0;
			double maximumChange = 0;

			for (int i1 = 0; i1 < discretization; i1++)
				for (int i2 = 0; i2 < discretization; i2++)
					for (int i3 = 0; i3 < discretization; i3++)
						for (int i4 = 0; i4 < discretization; i4++) 
							for (int i5 = 0; i5 < discretization; i5++) {
								// series[i][j].add(iterationCount, gw.states[i][j].getUtility());
								differences = Math.abs(states[i1][i2][i3][i4][i5].getUtility() - oldUtility[i1][i2][i3][i4][i5]);
								if (differences > maximumChange)
									maximumChange = differences;
							}
			return maximumChange;
		}
	}
	

	public static class ReservoirEnv extends Environment{
		
		DState[][][] states;
		public float oldUtility[][][];

		// Constructor
		public ReservoirEnv(int max_value, int discretization){
			max_value = max_value;
			discretization = discretization;
			ratio = (float) max_value / discretization;
			this.modifyStateSize();
		}

		protected void initiate(){
			int num_total_states = (int) Math.pow(discretization, dim_real_states) * (int) Math.pow(2, dim_binary_states);

			for (int i1 = 0; i1 < discretization; i1++)
				for (int i2 = 0; i2 < discretization; i2++)
					for (int i3 = 0; i3 < 2; i3++){
						states[i1][i2][i3] = new DState(this);
					}
					oldUtility = new float[discretization][discretization][2];			
		}

		public void modifyStateSize(){
			states = new DState[discretization][discretization][2];
			initiate();
		}

		// U(s) = R(s) + discount*MAX(expected utility of an action)
		public void updateUtility() {
			float actionUtility[] = new float[2];
			// double rewards[] = new double[2];
			int[] nextState;
			int bestAction;
			int action;

			// FOREACH STATE UPDATE THE UTILITY
			for (int i1 = 0; i1 < discretization; i1++)
				for (int i2 = 0; i2 < discretization; i2++)
					for (int i3 = 0; i3 < 2; i3++){
						// TODO: which exceptional cases should be handled?
						if (i1 * ratio < 20)
							continue;

						Arrays.fill(actionUtility, 0);
						// Arrays.fill(rewards, 0);
						
						// ACTION: 0 (block the flow in between two reservoirs)
						action = 0;
						nextState = getNextState(new int[]{i1, i2, i3}, action);
						nextState[2] = 0;
						actionUtility[action] = (float)(1-PROB_RAIN) * (reward + ((float) DISCOUNT) * oldUtility[nextState[0]][nextState[1]][nextState[2]]);
						nextState[2] = 1;
						actionUtility[action] += (float)PROB_RAIN * (reward + ((float) DISCOUNT) * oldUtility[nextState[0]][nextState[1]][nextState[2]]);
						// rewards[WE] = reward;

						// ACTION: 1 (allow the flow in between two reservoirs)
						action = 1;
						nextState = getNextState(new int[]{i1, i2, i3}, action);
						nextState[2] = 0;											// No rain at next state
						actionUtility[action] = (float) (1-PROB_RAIN) * (reward + ((float) DISCOUNT) * oldUtility[nextState[0]][nextState[1]][nextState[2]]);
						nextState[2] = 1;											// Rains at next state
						actionUtility[action] += (float) PROB_RAIN * (reward + ((float) DISCOUNT) * oldUtility[nextState[0]][nextState[1]][nextState[2]]);
						// rewards[NS] = reward;

						// SET THE ACTION WITH HIGHEST EXPECTED UTILITY
						bestAction = actionUtility[0] > actionUtility[1] ? 0 : 1;
						states[i1][i2][i3].setBestAction(bestAction);

						// UPDATE UTILITY BASED ON BELLMAN EQUATION
						// tr.states[i1][i2][i3][i4][i5].setUtility(rewards[bestAction] + DISCOUNT * actionUtility[bestAction]);
						states[i1][i2][i3].setUtility(actionUtility[bestAction]);
				}
		}

		public int[] optimizedTransition(int[] current_state, int action){
		// real_states = (l1, l2), binary_states = (r). 
		// We need (l1, l2, r, action) to determine the next state.
		// action = 0: block the water flow, i.e., q1 = 0. 
		// action = 1: q1 >= 0
		float l1, l2;
		float q1, q2;
		int[] next_state = new int[3];
		int[] _tempState, state_action_pair;

		int rain;

		// Check cache
		state_action_pair = new int[]{current_state[0], current_state[1], current_state[2], action};
		_tempState = _hmState2NextState.get(state_action_pair);
		if (_tempState != null){
			return _tempState;
		}

		l1 = getStateFromIndex(current_state[0]);
		l2 = getStateFromIndex(current_state[1]);
		rain = current_state[2];

		try {
			// An LP solver object initialized with 2 variables (no constraints)
			LpSolve solver = LpSolve.makeLp(0, 2);	
			
			// Add constraints
			double rhs1, rhs2, rhs3, rhs4;
			rhs1 = 0.98 * l1 + 200 * rain - 1000;
			rhs2 = 0.98 * l1 + 200 * rain - 3000;
			rhs3 = 700 - 0.98 * l2 - 200 * rain;
			rhs4 = 1500 - 0.98 * l2 - 200 * rain;
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

			// Need to maximize
			solver.setMaxim();

			// Solve the LP
			solver.solve();

			// Retrieve solution
			double[] var = solver.getPtrVariables();

			q1 = (float)var[0];
			q2 = (float)var[1];

			l1 = (float) (0.98 * l1 - q1 + 200 * rain);
			l2 = (float) (0.98 * l2 + q1 - q2 + 200 * rain);
			
			// delete the problem and free memory
			solver.deleteLp();

			// Compute the state transition and return the next state
			next_state[0] = getIndexFromState(l1);
			next_state[1] = getIndexFromState(l2);
			next_state[2] = current_state[2]; 				// TODO: stochastic transition?
			
			// Save the result in cache
			_hmState2NextState.put(state_action_pair, next_state);

			// Compute the reward
			reward = q2;

			return next_state;
		}
		catch (LpSolveException e) {
			e.printStackTrace();
		}
		return null;
	}
		public void storeOldUtility(){
			for (int i1 = 0; i1 < discretization; i1++)
				for (int i2 = 0; i2 < discretization; i2++)
					for (int i3 = 0; i3 < 2; i3++){
						oldUtility[i1][i2][i3] = states[i1][i2][i3].getUtility();
					}
		}

		public void setUtilityZero(){
			for (int i1 = 0; i1 < discretization; i1++)
				for (int i2 = 0; i2 < discretization; i2++)
					for (int i3 = 0; i3 < 2; i3++){
						states[i1][i2][i3].setUtility(0);
					}
		}

		public double computeMaximumDifference(){
			double differences = 0;
			double maximumChange = 0;

			for (int i1 = 0; i1 < discretization; i1++)
				for (int i2 = 0; i2 < discretization; i2++)
					for (int i3 = 0; i3 < 2; i3++){
						// series[i][j].add(iterationCount, gw.states[i][j].getUtility());
						differences = Math.abs(states[i1][i2][i3].getUtility() - oldUtility[i1][i2][i3]);
						if (differences > maximumChange)
							maximumChange = differences;
					}
			return maximumChange;
		}
	}

	public static class BandwidthEnv extends Environment{
		DState[][] states;
		public float oldUtility[][];
		// To be used in solving LP for Bandwidth problem
		public static HashMap<Integer, float[]> _hmAction2Ub = new HashMap<Integer, float[]>();
		public static HashMap<Integer, Float> _hmAction2BaseCost = new HashMap<Integer, Float>();
		public static float[] base_cost;
		public static float unit_cost;

		// Constructor
		public BandwidthEnv(int max_value, int discretization){
			max_value = max_value;
			discretization = discretization;
			ratio = (float) max_value / discretization;
			this.modifyStateSize();
		}

		protected void initiate(){
			int num_total_states = (int) Math.pow(discretization, dim_real_states) * (int) Math.pow(2, dim_binary_states);

			for (int i1 = 0; i1 < discretization; i1++)
				for (int i2 = 0; i2 < 2; i2++){
					states[i1][i2] = new DState(this);
				}
			oldUtility = new float[discretization][2];			
		}

		public void modifyStateSize(){
			states = new DState[discretization][2];
			initiate();
		}

		// U(s) = R(s) + discount*MAX(expected utility of an action)
		public void updateUtility() {
			float actionUtility[] = new float[7];
			// double rewards[] = new double[2];
			int[] nextState;
			int bestAction;
			float bestValue;

			// FOREACH STATE UPDATE THE UTILITY
			for (int i1 = 0; i1 < discretization; i1++)
				for (int i2 = 0; i2 < 2; i2++){
					// TODO: which exceptional cases should be handled?
					if (i1 * ratio < 20)
						continue;

					Arrays.fill(actionUtility, 0);
					// Arrays.fill(rewards, 0);
					
					bestValue = (float) -9E10;
					bestAction = -1;
					for (int a=0; a < 7; a++){
						nextState = getNextState(new int[]{i1, i2}, a);
						// Stays at the same demand level as the current state at next state
						nextState[1] = i2;
						actionUtility[a] = (float) PROB_SAME * (reward + ((float) DISCOUNT) * oldUtility[nextState[0]][nextState[1]]);
						
						// Demand level changes from the current state
						nextState[1] = (i2 == 0)? 1: 0;
						actionUtility[a] = (float) (1 - PROB_SAME) * (reward + ((float) DISCOUNT) * oldUtility[nextState[0]][nextState[1]]);
						
						// Low demand at next state
						if (actionUtility[a] > bestValue){
							bestValue = actionUtility[a];
							bestAction = a;
						}
					}
					states[i1][i2].setBestAction(bestAction);

					// UPDATE UTILITY BASED ON BELLMAN EQUATION
					// tr.states[i1][i2][i3][i4][i5].setUtility(rewards[bestAction] + DISCOUNT * actionUtility[bestAction]);
					states[i1][i2].setUtility(actionUtility[bestAction]);
			}
		}

		public int[] optimizedTransition(int[] current_state, int action){
			float ub;
			float next_demand, dnew;
			float demand;
			int level;
			int[] next_state = new int[2];
			int[] _tempState, state_action_pair;
	
			// Check cache
			state_action_pair = new int[]{current_state[0], current_state[1], action};
			_tempState = _hmState2NextState.get(state_action_pair);
			if (_tempState != null){
				return _tempState;
			}
	
			// Obtain maximum capacity of purchased edges
			float[] upper_bounds = _hmAction2Ub.get(action);
	
			// Get state value
			demand = getStateFromIndex(current_state[0]);
			level = current_state[1];
	
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
				solver.strSetObjFn("1 1 0 0 0");		// obj = xo1 + xo2
	
				// Need to maximize
				solver.setMaxim();
	
				// Solve the LP
				solver.solve();
	
				// Retrieve solution
				float max_flow = (float)solver.getObjective();
	
				// Compute the state transition and return the next state
				dnew = (level == 0)? 1200: 2500;
				if (demand > max_flow){
					next_demand = demand - max_flow + dnew;
				} else {
					next_demand = dnew;
				}
				next_state[0] = getIndexFromState(next_demand);
				next_state[1] = current_state[1];					// TODO: stochastic transition?
	
				// Save the result in cache
				_hmState2NextState.put(state_action_pair, next_state);
	
				// Retrieve total base cost incurred by action
				float total_base_cost = _hmAction2BaseCost.get(action);
	
				// Compute the reward
				float routed = Math.min(current_state[0], max_flow);
				reward = 6 * routed - 2 * Math.max(0, current_state[0] - max_flow);
				reward -= total_base_cost;
				reward -= unit_cost * max_flow;
				return next_state;
			}
			catch (LpSolveException e){
				e.printStackTrace();
			}
			return null;
		}
		
		public void storeOldUtility(){
			for (int i1 = 0; i1 < discretization; i1++)
				for (int i2 = 0; i2 < 2; i2++){
					oldUtility[i1][i2] = states[i1][i2].getUtility();
				}
		}

		public void setUtilityZero(){
			for (int i1 = 0; i1 < discretization; i1++)
				for (int i2 = 0; i2 < 2; i2++){
					states[i1][i2].setUtility(0);
				}			
		}

		public double computeMaximumDifference(){
			double differences = 0;
			double maximumChange = 0;

			for (int i1 = 0; i1 < discretization; i1++)
				for (int i2 = 0; i2 < 2; i2++){
					// series[i][j].add(iterationCount, gw.states[i][j].getUtility());
					differences = Math.abs(states[i1][i2].getUtility() - oldUtility[i1][i2]);
					if (differences > maximumChange)
						maximumChange = differences;
				}
			return maximumChange;
		}

		public void setupBandwidth(){
			float[] capacity = {2100, 1800, 1000, 1500, 1700};
			float[][] edgesInAction = {{1, 0, 0, 1, 0}, {1, 0, 1, 0, 1}, 
										{0, 1, 0, 0, 1}, {1, 0, 1, 1, 1}, 
										{1, 1, 0, 1, 1}, {1, 1, 1, 0, 1}, {1, 1, 1, 1, 1}};
			base_cost = new float[] {1000, 800, 600, 750, 800};
			unit_cost = (float) 1.3;
			float[] upper_bounds =  new float[5];
			Float total_base_cost;
	
			for (int a=0; a<7; a++){
				total_base_cost = (float)0;
				for (int i=0; i<5; i++){
					upper_bounds[i] = capacity[i] * edgesInAction[a][i];
					total_base_cost += base_cost[i] * edgesInAction[a][i];
				}
				_hmAction2BaseCost.put(a, total_base_cost);
				_hmAction2Ub.put(a, upper_bounds);
			}
		}
	}
}
