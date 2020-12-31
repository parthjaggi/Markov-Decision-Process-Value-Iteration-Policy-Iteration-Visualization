package discretized;

import com.google.gson.Gson;

import org.junit.internal.RealSystem;

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

	public static abstract class Environment implements Constant{
		public static int discretization;
		public static ArrayList<Float> min_values = new ArrayList<Float>();
		public static ArrayList<Float> max_values = new ArrayList<Float>();
		public static ArrayList<Float> ratios = new ArrayList<Float>();

		public static int num_real_states;
		public static int num_binary_states;
		public static int dim_binary_states;
		public static int dim_real_states;
		public static String _domain;
		public static float reward;
		public static int _status;

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
		public abstract String toJson();
		// public abstract String toJson2();

		public void setMinMaxValues(String[] args, int discretization){
			int num_var;
			if (this instanceof TrafficEnv){
				num_var = 5;
			}
			else if (this instanceof ReservoirEnv){
				num_var = 2;
			}
			else{
				num_var = 1;
			}

			min_values = new ArrayList<Float>();
			max_values = new ArrayList<Float>();
			ratios = new ArrayList<Float>();
			
			for (int i=0; i < num_var; i++){
				float min_val = Float.parseFloat(args[2 * i]);
				float max_val = Float.parseFloat(args[2 * i + 1]);
				min_values.add(min_val);
				max_values.add(max_val);
				ratios.add((float) (max_val - min_val) / (discretization - 1));
			}
		}

		public void setEnvName(){
			if (this instanceof TrafficEnv){
				_domain = "traffic";
			} else if (this instanceof ReservoirEnv){
				_domain = "reservoir";
			} else {
				_domain = "bandwidth";
			}
		}

		public int getDiscretization() {
			return discretization;
		}
	
		// public int getMaxValue() {
		// 	return max_value;
		// }
	
		// public float getRatio() {
		// 	return ratio;
		// }
		public float[] getStateFromIndex(int[] indices, int start_index, int end_index){
			/*
			ratio = (max_value - min_value) / discretization
			state_val = min_value + (max_value - min_value) * (i / discretization)
			when i = 0: state_val == min_value;
			when i = discretization: state_val = max_value;
			*/
			if (indices.length != end_index - start_index){
				System.out.println("Dimension mismatch! Exiting..");
				System.exit(1);
			}

			float[] states = new float[indices.length];
			int j = 0;
			for (int i=start_index; i<end_index; i++){
				states[j] = min_values.get(i) + indices[j] * ratios.get(i);
				j++;
			}
			return states;
			// return (float)(i * ratio);
		}

		public float[] getStateFromIndex(int[] indices){
			/*
			ratio = (max_value - min_value) / discretization
			state_val = min_value + (max_value - min_value) * (i / discretization)
			when i = 0: state_val == min_value;
			when i = discretization: state_val = max_value;
			*/
			float[] states = new float[indices.length];
			for (int i=0; i<indices.length; i++){
				states[i] = min_values.get(i) + indices[i] * ratios.get(i);
			}
			return states;
			// return (float)(i * ratio);
		}

		public int[] getIndexFromState(float[] states){
			/*
			ratio = (max_value - min_value) / discretization
			state_val = min_value + i * ratio;
			i = (state_val - min_value) / ratio;
			*/
			int[] indices = new int[states.length];
			// (int)Math.round((float) (q - min_value) / ratio);
			for (int i=0; i < states.length; i++){
				indices[i] = (int) Math.round((float) (states[i] - min_values.get(i)) / ratios.get(i));
				if (indices[i] >= discretization){
					indices[i] = discretization - 1;
				}
			}
			return indices;
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
		public TrafficEnv(int discretization){
			super.discretization = discretization;
			// super.ratio = (float) (max_value - min_value) / discretization;
			this.modifyStateSize();
		}

		public String toJson(){
			Gson gson = new Gson();
			return gson.toJson(oldUtility);
		}

		
		// public String toJson2() {
		// 	// Created for feasibility graphs.
		// 	Gson gson = new Gson();
		// 	return gson.toJson(oldUtility);
		// }

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
								// if (i1 * ratio < 20)
								// 	continue;


								Arrays.fill(actionUtility, 0);
								// Arrays.fill(rewards, 0);
								
								// ACTION: WE
								nextState = getNextState(new int[]{i1, i2, i3, i4, i5}, WE);
								if (nextState.length != 0){
									actionUtility[WE] = reward + ((float) DISCOUNT) * oldUtility[nextState[0]][nextState[1]][nextState[2]][nextState[3]][nextState[4]];
								}

								// ACTION: NS
								nextState = getNextState(new int[]{i1, i2, i3, i4, i5}, NS);
								if (nextState.length != 0){
									actionUtility[NS] = reward + ((float) DISCOUNT) * oldUtility[nextState[0]][nextState[1]][nextState[2]][nextState[3]][nextState[4]];
								}
								
								// SET THE ACTION WITH HIGHEST EXPECTED UTILITY
								bestAction = actionUtility[WE] > actionUtility[NS] ? WE : NS;
								states[i1][i2][i3][i4][i5].setBestAction(bestAction);

								// UPDATE UTILITY BASED ON BELLMAN EQUATION
								// tr.states[i1][i2][i3][i4][i5].setUtility(rewards[bestAction] + DISCOUNT * actionUtility[bestAction]);
								states[i1][i2][i3][i4][i5].setUtility(actionUtility[bestAction]);			
							}
		}

		public float determineNumVehTransfer(float q4, float q5){
			if ((q4 > 15) && (q5 < max_values.get(4) - 15)){
				return 15;
			} else if ((q4 > 15) && (q5 >= max_values.get(4) - 15)){
				return max_values.get(4) - q5;
			} else if ((q4 <= 15) && (q5 < max_values.get(4) - q4)){
				return q4;
			} else {
				return max_values.get(4) - q5;
			}
		}

		public int[] optimizedTransition(int[] current_state, int action){
			float q1, q2, q3, q4, q5;
			float dq2, dq3;
			int[] next_state = new int[5];
			int[] _tempStates;
			float[] real_state = new float[3];

			if (action == 1){
				float[] q4q5 = getStateFromIndex(Arrays.copyOfRange(current_state, 3, 5), 3, 5);
				q4 = q4q5[0]; q5 = q4q5[1];
				float numToTransfer = determineNumVehTransfer(q4, q5);
				int idxToTransfer = Math.round(numToTransfer / ratios.get(4));		// Assuming q4 and q5 ratios are the same

				// int idxToTransfer = Math.min(Math.min(Math.max(Math.round(current_state[3]), 0), Math.round(discretization - current_state[4] - 1)), (int) (15 / ratios.get(3))); 	
				next_state[0] = current_state[0];		// q1, q2, q3 remain the same
				next_state[1] = current_state[1];
				next_state[2] = current_state[2];
				next_state[3] = Math.round(current_state[3] - idxToTransfer);
				next_state[4] = Math.min(current_state[4] + idxToTransfer, discretization - 1);
				reward = numToTransfer;
				return next_state;
			}
	
			_tempStates = new int[] {current_state[0], current_state[1], current_state[2]};
	
			// Check cache
			_tempStates = _hmState2NextState.get(_tempStates);
			if (_tempStates != null){
				return _tempStates;
			}
			
			real_state = getStateFromIndex(Arrays.copyOfRange(current_state, 0, 3));
			q1 = real_state[0];
			q2 = real_state[1];
			q3 = real_state[2];
	
			try {
				// An LP solver object initialized with 2 variables (no constraints)
				LpSolve solver = LpSolve.makeLp(0, 2);	
				
				// Verbose level
				solver.setVerbose(LpSolve.CRITICAL);
	
				// Add constraints
				solver.strAddConstraint("1 1", LpSolve.LE, 20);		// dq2 + dq3 <= 20
				solver.strAddConstraint("1 0", LpSolve.LE, 120-q2);	// dq2 		 <= 120 - q2
				solver.strAddConstraint("0 1", LpSolve.LE, 100-q3);	// 		 dq3 <= 100 - q3
				solver.strAddConstraint("1 1", LpSolve.LE, q1);		// dq2 + dq3 <= q1
				solver.strAddConstraint("1 0", LpSolve.GE, 0);
				solver.strAddConstraint("0 1", LpSolve.GE, 0);		// Bound constraints
				
				
				// Set objective function
				solver.strSetObjFn("1 1");	// obj = dq2 + dq3
	
				// Need to maximize
				solver.setMaxim();
	
				// Solve the problem
				_status = solver.solve();
	
				// Handle infeasible cases
				if (_status == LpSolve.INFEASIBLE){
					return new int[] {};
				}

				// Retrieve solution
				double[] var = solver.getPtrVariables();
	
				dq2 = (float)var[0];
				dq3 = (float)var[1];
				
				q2 += dq2;
				q3 += dq3;
				q1 -= (dq2 + dq3);
				
				_tempStates = getIndexFromState(new float[]{q1, q2, q3});
				next_state[0] = _tempStates[0];
				next_state[1] = _tempStates[1];
				next_state[2] = _tempStates[2];
				next_state[3] = current_state[3];					// q4, q5 remain the same
				next_state[4] = current_state[4];

				// delte the problem and free memory
				solver.deleteLp();
	
				// Save the result in cache
				_hmState2NextState.put(current_state, next_state);
	
				// Compute reward
				// reward = (current_state[0] - next_state[0]) * ratio;
				reward = dq2 + dq3;
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
		// public float isFeasible[][];
		public float oldUtility[][][];

		// Constructor
		public ReservoirEnv(int discretization){
			super.discretization = discretization;
			// super.ratio = (float) (max_value - min_value) / discretization;
			// super.ratio2 = (float) (max_value2 - min_value2) / discretization;
			this.modifyStateSize();
		}

		public String toJson(){
			Gson gson = new Gson();
			return gson.toJson(oldUtility);
		}

		// public String toJson2() {
		// 	// Created for feasibility graphs.
		// 	Gson gson = new Gson();
		// 	return gson.toJson(isFeasible);
		// }

		protected void initiate(){
			for (int i1 = 0; i1 < discretization; i1++)
				for (int i2 = 0; i2 < discretization; i2++)
					for (int i3 = 0; i3 < 2; i3++){
						states[i1][i2][i3] = new DState(this);
					}
					oldUtility = new float[discretization][discretization][2];			
					// isFeasible = new float[discretization][discretization];
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
						// If the water level of any of the reservoir goes out of the domain constraints, do nothing
						float[] current_state = getStateFromIndex(new int[]{i1, i2});
						// if ((current_state[0] < 1000) || (current_state[1] < 700) || (current_state[0] > 3000) || (current_state[1] > 1500)){
						// 	continue;
						// }

						// Initialize actionutility array
						Arrays.fill(actionUtility, 0);
						
						// ACTION: 0 (block the flow in between two reservoirs)
						action = 0;
						nextState = getNextState(new int[]{i1, i2, i3}, action);
						if (nextState.length != 0){
							nextState[2] = 0;											// no rain
							actionUtility[action] = (float)(1-PROB_RAIN) * (reward + ((float) DISCOUNT) * oldUtility[nextState[0]][nextState[1]][nextState[2]]);
							nextState[2] = 1;											// rain
							actionUtility[action] += (float)PROB_RAIN * (reward + ((float) DISCOUNT) * oldUtility[nextState[0]][nextState[1]][nextState[2]]);
						}
						
						// ACTION: 1 (allow the flow in between two reservoirs)
						action = 1;
						nextState = getNextState(new int[]{i1, i2, i3}, action);
						if (nextState.length != 0){
							nextState[2] = 0;											// No rain at next state
							actionUtility[action] = (float) (1-PROB_RAIN) * (reward + ((float) DISCOUNT) * oldUtility[nextState[0]][nextState[1]][nextState[2]]);
							nextState[2] = 1;											// Rains at next state
							actionUtility[action] += (float) PROB_RAIN * (reward + ((float) DISCOUNT) * oldUtility[nextState[0]][nextState[1]][nextState[2]]);
						}
						// if (_status == LpSolve.INFEASIBLE){
						// 	isFeasible[i1][i2] = (float) 0.0;
						// } else {
						// 	isFeasible[i1][i2] = (float) 1.0;
						// }
						
						// SET THE ACTION WITH HIGHEST EXPECTED UTILITY
						bestAction = actionUtility[0] > actionUtility[1] ? 0 : 1;
						states[i1][i2][i3].setBestAction(bestAction);

						// UPDATE UTILITY BASED ON BELLMAN EQUATION
						states[i1][i2][i3].setUtility(actionUtility[bestAction]);
				}
		}

		public int[] optimizedTransition(int[] current_state, int action){
		// real_states = (l1, l2), binary_states = (r). 
		// We need (l1, l2, r, action) to determine the next state.
		// action = 0: block the water flow, i.e., q1 = 0. 
		// action = 1: q1 >= 0
		float[] level;
		float l1, l2;
		float q1, q2;
		int[] next_level = new int[2];
		int[] next_state = new int[3];
		int[] _tempState, state_action_pair;

		int rain;

		// Check cache
		state_action_pair = new int[]{current_state[0], current_state[1], current_state[2], action};
		_tempState = _hmState2NextState.get(state_action_pair);
		if (_tempState != null){
			return _tempState;
		}

		level = getStateFromIndex(new int[] {current_state[0], current_state[1]});
		l1 = level[0];
		l2 = level[1];
		rain = current_state[2];

		try {
			// An LP solver object initialized with 2 variables (no constraints)
			LpSolve solver = LpSolve.makeLp(0, 2);	
			
			// Verbose level
			solver.setVerbose(LpSolve.CRITICAL);

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
			solver.strAddConstraint("1 0", LpSolve.GE, 0);
			solver.strAddConstraint("0 1", LpSolve.GE, 0);
			solver.strAddConstraint("1 0", LpSolve.LE, (double) 250 * action);
			solver.strAddConstraint("0 1", LpSolve.LE, 300);

			// Set objective function
			solver.strSetObjFn("0 1");		// obj = q2

			// Need to maximize
			solver.setMaxim();

			// Solve the LP
			_status = solver.solve();

			// Handle infeasible cases
			if (_status == LpSolve.INFEASIBLE){
				return new int[] {};
			}

			// Retrieve solution
			double[] var = solver.getPtrVariables();

			q1 = (float)var[0];
			q2 = (float)var[1];

			l1 = (float) (0.98 * l1 - q1 + 200 * rain);
			l2 = (float) (0.98 * l2 + q1 - q2 + 200 * rain);
			
			// delete the problem and free memory
			solver.deleteLp();

			// Compute the state transition and return the next state
			level = new float[] {l1, l2};
			next_level = getIndexFromState(level);
			next_state[0] = next_level[0];
			next_state[1] = next_level[1];
			next_state[2] = current_state[2]; 				// Note: stochastic transition dealt with in updateUtility method
			
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
		public static double _DISCOUNT = 0.95;

		// Constructor
		public BandwidthEnv(int discretization){
			super.discretization = discretization;
			this.modifyStateSize();
			setupBandwidth();
		}

		public String toJson(){
			Gson gson = new Gson();
			return gson.toJson(oldUtility);
		}

		protected void initiate(){
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
					// All nonnegative states are valid
					// if (i1 * ratio < 20)
					// 	continue;

					Arrays.fill(actionUtility, 0);
					// Arrays.fill(rewards, 0);
					
					bestValue = (float) -9E10;
					bestAction = -1;
					for (int a=0; a < 7; a++){
						nextState = getNextState(new int[]{i1, i2}, a);
						if (nextState.length != 0){
							// s' = s: Stays at the same demand level as the current state at next state
							// nextState[1] = i2;
							// actionUtility[a] = (float) PROB_SAME * (reward + ((float) _DISCOUNT) * oldUtility[nextState[0]][nextState[1]]);
							
							nextState[1] = 1;	// High demand
							actionUtility[a] = (float) PROB_HIGH * (reward + ((float) _DISCOUNT) * oldUtility[nextState[0]][nextState[1]]);

							// s' != s: Demand level changes from the current state
							// nextState[1] = (i2 == 0)? 1: 0;
							// actionUtility[a] += (float) (1 - PROB_SAME) * (reward + ((float) _DISCOUNT) * oldUtility[nextState[0]][nextState[1]]);

							nextState[1] = 0;	// Low demand
							actionUtility[a] += (float) (1 - PROB_HIGH) * (reward + ((float) _DISCOUNT) * oldUtility[nextState[0]][nextState[1]]);


							if (actionUtility[a] > bestValue){
								bestValue = actionUtility[a];
								bestAction = a;
							}
						}
						else{
							continue;
						}
					}

					if (bestAction != -1){
						states[i1][i2].setBestAction(bestAction);

						// UPDATE UTILITY BASED ON BELLMAN EQUATION
						// tr.states[i1][i2][i3][i4][i5].setUtility(rewards[bestAction] + DISCOUNT * actionUtility[bestAction]);
						states[i1][i2].setUtility(actionUtility[bestAction]);
					}
					else{
						states[i1][i2].setUtility(0);
					}
					
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
			demand = getStateFromIndex(new int[] {current_state[0]})[0];
			level = current_state[1];
	
			try {
				// An LP solver object initialized with 5 variables (no constraints)
				LpSolve solver = LpSolve.makeLp(0, 5);

				// Set verbosity level
				solver.setVerbose(LpSolve.CRITICAL);

				// Add constraints
				solver.strAddConstraint("1 0 -1 -1 0", LpSolve.EQ, 0);
				solver.strAddConstraint("0 -1 -1 0 1", LpSolve.EQ, 0);
	
				
				int[] var_int = new int[5];
				// Bound constraints
				for (int i=0; i<5; i++){
					Arrays.fill(var_int, 0);
					var_int[i] = 1;
					String var_str = "";
					for (int j=0; j<5; j++){
						var_str += String.format("%d ", var_int[j]);
					}
					var_str = var_str.substring(0, var_str.length()-1);

					ub = upper_bounds[i];
					solver.strAddConstraint(var_str, LpSolve.GE, 0);
					solver.strAddConstraint(var_str, LpSolve.LE, ub);
				}
				
				// Set objective function
				solver.strSetObjFn("1 1 0 0 0");		// obj = xo1 + xo2
	
				// Need to maximize
				solver.setMaxim();
	
				// Solve the LP
				_status = solver.solve();

				// Handle infeasibility
				if (_status == LpSolve.INFEASIBLE){
					return new int[] {};
				}
	
				// Retrieve solution
				float max_flow = (float)solver.getObjective();
	
				// Compute the state transition and return the next state
				dnew = (level == 0)? 1200: 2500;
				if (demand > max_flow){
					next_demand = demand - max_flow + dnew;
				} else {
					next_demand = dnew;
				}
				next_state[0] = getIndexFromState(new float[] {next_demand})[0];
				next_state[1] = current_state[1];					// Note: stochastic transition is handled in updateUtility
	
				// Save the result in cache
				_hmState2NextState.put(state_action_pair, next_state);
	
				// Retrieve total base cost incurred by action
				float total_base_cost = _hmAction2BaseCost.get(action);
	
				// Compute the reward
				float routed = Math.min(demand, max_flow);
				reward = 6 * routed - 2 * Math.max(0, demand - max_flow);
				reward -= total_base_cost;
				reward -= unit_cost * routed;
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
			float total_base_cost;
	
			for (int a=0; a<7; a++){
				total_base_cost = (float)0;
				float[] upper_bounds =  new float[5];
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
