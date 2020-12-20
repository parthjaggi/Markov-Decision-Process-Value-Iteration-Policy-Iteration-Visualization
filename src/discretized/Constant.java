package discretized;

/**
 * @author GOH KA HIAN NANYANG TECHNOLOGICAL UNIVERSITY
 *
 */
public interface Constant {
	// DISCOUNT
	// static final double DISCOUNT = 0.99;
	static final double DISCOUNT = 1.0;

	// ACTIONS-TRAFFIC
	static final int WE = 0;
	static final int NS = 1;

	// Transition probabilities for reservoir domain
	static final double PROB_RAIN = 0.4;

	// Transition probabilities for bandwidth domain
	static final double PROB_SAME = 0.7;

}
