
import java.util.Arrays;
import java.awt.Dimension;
import org.jfree.data.xy.XYSeries;
import org.jfree.data.xy.XYSeriesCollection;

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

		System.out.println("VI has converged");

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
			// j1, j2, j3 = solveLP(i1, i2, i3)
			// i4, i5 remain the same.
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
