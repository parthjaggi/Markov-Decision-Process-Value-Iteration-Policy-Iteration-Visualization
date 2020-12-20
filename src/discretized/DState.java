package discretized;
/**
 *
 */
public class DState implements Constant {
	/**
	 * 
	 */
	private float reward;
	// 0 = MOVE UP, 1 = MOVE DOWN, 2 = MOVE LEFT, 3 = MOVE RIGHT
	private float utility;
	private int bestAction;
	public Object env;

	// CONSTRUCTOR
	public DState(Object env) {
		this.env = env;
	}

	// START OF GETTER AND SETTER
	public float getReward() {
		return reward;
	}

	public void setReward(float reward) {
		this.reward = reward;
	}

	public int getBestAction() {
		return bestAction;
	}

	public void setBestAction(int bestAction) {
		this.bestAction = bestAction;
	}

	public float getUtility() {
		return utility;
	}

	public void setUtility(float utility) {
		this.utility = utility;
	}
}
