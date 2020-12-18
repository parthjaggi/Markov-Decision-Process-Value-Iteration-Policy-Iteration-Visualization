
import java.lang.Math;
import java.awt.GridLayout;

import javax.swing.BorderFactory;
import javax.swing.JPanel;

/**
 * @author Humpty Dumpty.
 *
 */
public class TrafficEnv implements Constant {
	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;
	// X-AXIS
	private int cols;
	// Y-AXIS
	private int rows;
	private int discretization;
	private int max_value;
	private float ratio;

	protected JPanel[][] entireMap;
	protected StateT[][][][][] states;

	// CONSTRUCTOR
	public TrafficEnv() {
		// stateSizeChange();
	}

	public TrafficEnv(int max_value, int discretization) {
		this.max_value = max_value;
		this.discretization = discretization;
		ratio = max_value / discretization;
		// this.cols = cols;
		// stateSizeChange();
	}

	// SET COLS AND ROWS
	public void setColsRowsSize(int xy) {
		this.rows = xy;
		this.cols = xy;
	}

	// INITIAL STATES OBJECT AND MAP
	protected void initiate() {
		// this.removeAll();
		// this.revalidate();
		// this.repaint();
		// this.setLayout(new GridLayout(cols, rows));
		for (int i1 = 0; i1 < discretization; i1++)
			for (int i2 = 0; i2 < discretization; i2++)
				for (int i3 = 0; i3 < discretization; i3++)
					for (int i4 = 0; i4 < discretization; i4++) 
						for (int i5 = 0; i5 < discretization; i5++) {
							states[i1][i2][i3][i4][i5] = new StateT(this);
							// states[i][j].setBorder(BorderFactory.createLineBorder(BORDER, 1));
							// entireMap[i][j] = states[i][j];
						}

		// DIFFERENT FORMAT FOR GUI (ROWS,COLS) INSTEAD OF (COLS,ROWS)
		// for (int i = 0; i < cols; i++)
		// 	for (int j = 0; j < rows; j++)
		// 		this.add(states[j][i]);
	}

	// PERFORM UPDATE OF STATE SIZE AND MAP
	void modifyStateSize() {
		// entireMap = new JPanel[cols][rows];
		states = new StateT[discretization][discretization][discretization][discretization][discretization];
		initiate();
		// updateIsWall();
	}

	// THERE IS A CHANGE IN STATE SIZE, UPDATE
	// void stateSizeChange() {
	// 	modifyStateSize();
	// 	if (!defaultConfig()) {
	// 		randomizeConfig();
	// 	}
	// }

	// SET ALL STATES TO 'WHITE'
	// void setEmpty() {
	// 	for (int i = 0; i < cols; i++)
	// 		for (int j = 0; j < rows; j++)
	// 			states[i][j].setColor(WHITE);
	// }

	// DISPLAY OPTIMAL POLICY
	public void displayOptimalPolicy() {
		for (int i1 = 0; i1 < discretization; i1++)
			for (int i2 = 0; i2 < discretization; i2++)
				for (int i3 = 0; i3 < discretization; i3++)
					for (int i4 = 0; i4 < discretization; i4++) 
						for (int i5 = 0; i5 < discretization; i5++) {
							// if (states[i][j].isWall())
							// 	continue;

							if (states[i1][i2][i3][i4][i5].getBestAction() == WE)
								states[i1][i2][i3][i4][i5].getText().setText("WE");
							else if (states[i1][i2][i3][i4][i5].getBestAction() == NS)
								states[i1][i2][i3][i4][i5].getText().setText("NS");
				
							// Main.displayRewardButton.setEnabled(true);
							// Main.displayUtilityButton.setEnabled(true);
							// Main.displayPolicyButton.setEnabled(false);
						}
	}

	// DISPLAY REWARD FOR STATES
	// public void displayReward() {
	// 	for (int i = 0; i < cols; i++) {
	// 		for (int j = 0; j < rows; j++) {
	// 			if (states[i][j].isWall())
	// 				continue;
	// 			states[i][j].getText().setText(Double.toString(states[i][j].getReward()));
	// 			Main.displayRewardButton.setEnabled(false);
	// 			Main.displayUtilityButton.setEnabled(true);
	// 			Main.displayPolicyButton.setEnabled(true);
	// 		}
	// 	}
	// }

	// DISPLAY UTILITY 4 DECIMAL PLACING
	// public void displayUtility() {
	// 	for (int i = 0; i < cols; i++) {
	// 		for (int j = 0; j < rows; j++) {
	// 			if (states[i][j].isWall())
	// 				continue;
	// 			states[i][j].getText().setText(

	// 					Double.toString((double) Math.round(states[i][j].getUtility() * 10000) / 10000));

	// 			Main.displayRewardButton.setEnabled(true);
	// 			Main.displayUtilityButton.setEnabled(false);
	// 			Main.displayPolicyButton.setEnabled(true);
	// 		}
	// 	}
	// }

	// START OF GETTERS AND SETTERS
	public int getCols() {
		return cols;
	}

	public void setCols(int cols) {
		this.cols = cols;
	}

	public int getRows() {
		return rows;
	}

	public int getDiscretization() {
		return discretization;
	}

	public int getMaxValue() {
		return max_value;
	}

	public float getRatio() {
		return ratio;
	}

	public void setRows(int rows) {
		this.rows = rows;
	}

}
