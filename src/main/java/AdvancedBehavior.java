package main.java;
import burlap.behavior.policy.GreedyQPolicy;
import burlap.domain.singleagent.gridworld.GridWorldTerminalFunction;
import burlap.domain.singleagent.gridworld.GridWorldRewardFunction;
import burlap.behavior.policy.Policy;
import burlap.behavior.singleagent.EpisodeAnalysis;
import burlap.behavior.singleagent.auxiliary.EpisodeSequenceVisualizer;
import burlap.behavior.singleagent.auxiliary.StateReachability;
import burlap.behavior.singleagent.auxiliary.performance.LearningAlgorithmExperimenter;
import burlap.behavior.singleagent.auxiliary.performance.PerformanceMetric;
import burlap.behavior.singleagent.auxiliary.performance.TrialMode;
import burlap.behavior.singleagent.auxiliary.valuefunctionvis.ValueFunctionVisualizerGUI;
import burlap.behavior.singleagent.auxiliary.valuefunctionvis.common.ArrowActionGlyph;
import burlap.behavior.singleagent.auxiliary.valuefunctionvis.common.LandmarkColorBlendInterpolation;
import burlap.behavior.singleagent.auxiliary.valuefunctionvis.common.PolicyGlyphPainter2D;
import burlap.behavior.singleagent.auxiliary.valuefunctionvis.common.StateValuePainter2D;
import burlap.behavior.singleagent.learning.LearningAgent;
import burlap.behavior.singleagent.learning.LearningAgentFactory;
import burlap.behavior.singleagent.learning.tdmethods.QLearning;
import burlap.behavior.singleagent.learning.tdmethods.SarsaLam;
import burlap.behavior.singleagent.planning.Planner;
import burlap.behavior.singleagent.planning.deterministic.DeterministicPlanner;
import burlap.behavior.singleagent.planning.deterministic.informed.Heuristic;
import burlap.behavior.singleagent.planning.deterministic.informed.astar.AStar;
import burlap.behavior.singleagent.planning.deterministic.uninformed.bfs.BFS;
import burlap.behavior.singleagent.planning.deterministic.uninformed.dfs.DFS;
import burlap.behavior.singleagent.planning.stochastic.valueiteration.ValueIteration;
import burlap.behavior.singleagent.planning.stochastic.policyiteration.PolicyIteration;
import burlap.behavior.valuefunction.QFunction;
import burlap.behavior.valuefunction.ValueFunction;
import burlap.domain.singleagent.gridworld.GridWorldDomain;
import burlap.domain.singleagent.gridworld.GridWorldVisualizer;
import burlap.oomdp.auxiliary.common.SinglePFTF;
import burlap.oomdp.auxiliary.stateconditiontest.StateConditionTest;
import burlap.oomdp.auxiliary.stateconditiontest.TFGoalCondition;
import burlap.oomdp.core.Domain;
import burlap.oomdp.core.TerminalFunction;
import burlap.oomdp.core.objects.ObjectInstance;
import burlap.oomdp.core.states.State;
import burlap.oomdp.singleagent.RewardFunction;
import burlap.oomdp.singleagent.SADomain;
import burlap.oomdp.singleagent.common.GoalBasedRF;
import burlap.oomdp.singleagent.common.UniformCostRF;
import burlap.oomdp.singleagent.common.VisualActionObserver;
import burlap.oomdp.singleagent.environment.Environment;
import burlap.oomdp.singleagent.environment.EnvironmentServer;
import burlap.oomdp.singleagent.environment.SimulatedEnvironment;
import burlap.oomdp.statehashing.HashableStateFactory;
import burlap.oomdp.statehashing.SimpleHashableStateFactory;
import burlap.oomdp.visualizer.Visualizer;
import burlap.oomdp.auxiliary.DomainGenerator;
import burlap.oomdp.core.*;
import burlap.oomdp.core.objects.MutableObjectInstance;
import burlap.oomdp.core.objects.ObjectInstance;
import burlap.oomdp.core.states.MutableState;
import burlap.oomdp.core.states.State;
import burlap.oomdp.singleagent.*;
import burlap.oomdp.singleagent.common.SimpleAction;
import burlap.oomdp.singleagent.environment.SimulatedEnvironment;
import burlap.oomdp.singleagent.explorer.TerminalExplorer;
import burlap.oomdp.singleagent.explorer.VisualExplorer;
import burlap.oomdp.visualizer.ObjectPainter;
import burlap.oomdp.visualizer.StateRenderLayer;
import burlap.oomdp.visualizer.StaticPainter;
import burlap.oomdp.visualizer.Visualizer;

import java.awt.*;
import java.util.List;


public class AdvancedBehavior {

	GridWorldDomain gwdg;
	Domain domain;
	//RewardFunction rf;
    GridWorldRewardFunction rf;
	//TerminalFunction tf;
    GridWorldTerminalFunction tf;
	StateConditionTest goalCondition;
	State initialState;
	HashableStateFactory hashingFactory;
	Environment env;
    public static final String ATTX = "x";
	public static final String ATTY = "y";

	public static final String CLASSAGENT = "agent";
	public static final String CLASSLOCATION = "location";

	public static final String ACTIONNORTH = "north";
	public static final String ACTIONSOUTH = "south";
	public static final String ACTIONEAST = "east";
	public static final String ACTIONWEST = "west";

	public static final String PFAT = "at";
protected int [][] map = new int[][]{
			{0,0,0,0,0,0,0,0,0},
            {0,1,0,0,0,1,0,0,0},
			{0,1,0,1,1,1,1,0,1},
			{0,1,0,0,0,0,0,0,0},
            {0,1,1,1,1,1,1,0,0},
            {0,0,0,0,0,0,0,0,0},
            {1,1,1,0,1,1,0,0,0},
            {0,0,0,0,1,0,0,0,0},
            {0,0,0,0,0,0,0,0,0}
	};

	public AdvancedBehavior(){
		gwdg = new GridWorldDomain(9,9);
        System.out.println(map[0].length);
        System.out.println(map.length);
		gwdg.setMap(map);
        //gwdg.setProbSucceedTransitionDynamics(.9385);
        gwdg.setProbSucceedTransitionDynamics(0.8);
		domain = gwdg.generateDomain();


        rf = new GridWorldRewardFunction(domain, 0.0);
        rf.setReward(map[0].length - 1 ,map.length - 1,1.0);
        rf.setReward(map[0].length - 1 ,map.length - 1,-1.0);
        tf = new GridWorldTerminalFunction();
        tf.markAsTerminalPosition(map[0].length - 1 ,map.length - 1);
        tf.markAsTerminalPosition(map[0].length - 1 ,map.length - 1);
        
		goalCondition = new TFGoalCondition(tf);

		initialState = GridWorldDomain.getOneAgentNLocationState(domain, 1);
		GridWorldDomain.setAgent(initialState, 0, 0);
		GridWorldDomain.setLocation(initialState, 0, map[0].length - 1 ,map.length - 1);

		hashingFactory = new SimpleHashableStateFactory();

		env = new SimulatedEnvironment(domain, rf, tf, initialState);


		/*
		VisualActionObserver observer = new VisualActionObserver(domain, 
									GridWorldVisualizer.getVisualizer(gwdg.getMap()));
		observer.initGUI();
		env = new EnvironmentServer(env, observer);
		//((SADomain)domain).addActionObserverForAllAction(observer);
		*/
	}
public static class ExampleTF implements TerminalFunction{

	int goalX;
	int goalY;
	int avoidX;
	int avoidY;
    
	public ExampleTF(int goalX, int goalY, int avoidX, int avoidY){
		this.goalX = goalX;
		this.goalY = goalY;
        this.avoidX = avoidX;
		this.avoidY = avoidY;
	}
	
	@Override
	public boolean isTerminal(State s) {
		
		//get location of agent in next state
		ObjectInstance agent = s.getFirstObjectOfClass(CLASSAGENT);
		int ax = agent.getIntValForAttribute(ATTX);
		int ay = agent.getIntValForAttribute(ATTY);
		
		//are they at goal location?
		if(ax == this.goalX && ay == this.goalY){
			return true;
		}
		if(ax == this.avoidX && ay == this.avoidY){
			return true;
		}
		return false;
	}
	
	
	
}

    public static class ExampleRF implements RewardFunction{

	int goalX;
	int goalY;
	int avoidX;
	int avoidY;
    
	public ExampleRF(int goalX, int goalY, int avoidX, int avoidY){
		this.goalX = goalX;
		this.goalY = goalY;
        this.avoidX = avoidX;
		this.avoidY = avoidY;
	}
	
	@Override
	public double reward(State s, GroundedAction a, State sprime) {
		
		//get location of agent in next state
		ObjectInstance agent = sprime.getFirstObjectOfClass(CLASSAGENT);
		int ax = agent.getIntValForAttribute(ATTX);
		int ay = agent.getIntValForAttribute(ATTY);
		
		//are they at goal location?
		if(ax == this.goalX && ay == this.goalY){
			return 1.0;
		}
		if(ax == this.avoidX && ay == this.avoidY){
			return -1.0;
		}
		return 0;
	}
	
	
}

	public void visualize(String outputpath){
		Visualizer v = GridWorldVisualizer.getVisualizer(gwdg.getMap());
		new EpisodeSequenceVisualizer(v, domain, outputpath);
	}

	public void BFSExample(String outputPath){

		DeterministicPlanner planner = new BFS(domain, goalCondition, hashingFactory);
		Policy p = planner.planFromState(initialState);
		p.evaluateBehavior(initialState, rf, tf).writeToFile(outputPath + "bfs");

	}

	public void DFSExample(String outputPath){

		DeterministicPlanner planner = new DFS(domain, goalCondition, hashingFactory);
		Policy p = planner.planFromState(initialState);
		p.evaluateBehavior(initialState, rf, tf).writeToFile(outputPath + "dfs");

	}

	public void AStarExample(String outputPath){

		Heuristic mdistHeuristic = new Heuristic() {
			@Override
			public double h(State s) {

				ObjectInstance agent = s.getFirstObjectOfClass(GridWorldDomain.CLASSAGENT);
				ObjectInstance location = s.getFirstObjectOfClass(GridWorldDomain.CLASSLOCATION);

				int ax = agent.getIntValForAttribute(GridWorldDomain.ATTX);
				int ay = agent.getIntValForAttribute(GridWorldDomain.ATTY);

				int lx = location.getIntValForAttribute(GridWorldDomain.ATTX);
				int ly = location.getIntValForAttribute(GridWorldDomain.ATTY);

				double mdist = Math.abs(ax-lx) + Math.abs(ay-ly);

				return -mdist;
			}
		};

		DeterministicPlanner planner = new AStar(domain, rf, goalCondition, hashingFactory, mdistHeuristic);
		Policy p = planner.planFromState(initialState);

		p.evaluateBehavior(initialState, rf, tf).writeToFile(outputPath + "astar");

	}

	public void valueIterationExample(String outputPath){

		Planner planner = new ValueIteration(domain, rf, tf, 0.99, hashingFactory, 0.001, 1000);
		Policy p = planner.planFromState(initialState);

		p.evaluateBehavior(initialState, rf, tf).writeToFile(outputPath + "vi");

		//simpleValueFunctionVis((ValueFunction)planner, p);
		manualValueFunctionVis((ValueFunction)planner, p, "4x3 GridWorld Value Iteration");

	}
	public void policyIterationExample(String outputPath){

		Planner planner = new PolicyIteration(domain, rf, tf, 0.99, hashingFactory, 0.001, 1000, 1000);
		Policy p = planner.planFromState(initialState);

		p.evaluateBehavior(initialState, rf, tf).writeToFile(outputPath + "pi");

		//simpleValueFunctionVis((ValueFunction)planner, p);
		manualValueFunctionVis((ValueFunction)planner, p, "4x3 GridWorld Policy Iteration");

	}

	public void qLearningExample(String outputPath){

		LearningAgent agent = new QLearning(domain, 0.99, hashingFactory, 0., 1.);

		//run learning for 100 episodes
		for(int i = 0; i < 100; i++){
			EpisodeAnalysis ea = agent.runLearningEpisode(env);

			ea.writeToFile(outputPath + "ql_" + i);
			System.out.println(i + ": " + ea.maxTimeStep());

			//reset environment for next learning episode
			env.resetEnvironment();
		}

	}


	public void sarsaLearningExample(String outputPath){

		LearningAgent agent = new SarsaLam(domain, 0.99, hashingFactory, 0., 0.5, 0.3);

		//run learning for 50 episodes
		for(int i = 0; i < 50; i++){
			EpisodeAnalysis ea = agent.runLearningEpisode(env);

			ea.writeToFile(outputPath + "sarsa_" + i);
			System.out.println(i + ": " + ea.maxTimeStep());

			//reset environment for next learning episode
			env.resetEnvironment();
		}

	}

	public void simpleValueFunctionVis(ValueFunction valueFunction, Policy p){

		List<State> allStates = StateReachability.getReachableStates(initialState, 
									(SADomain)domain, hashingFactory);
		ValueFunctionVisualizerGUI gui = GridWorldDomain.getGridWorldValueFunctionVisualization(
											allStates, valueFunction, p);
		gui.initGUI();

	}

	public void manualValueFunctionVis(ValueFunction valueFunction, Policy p, String title){

		List<State> allStates = StateReachability.getReachableStates(initialState, 
									(SADomain)domain, hashingFactory);

		//define color function
		LandmarkColorBlendInterpolation rb = new LandmarkColorBlendInterpolation();
		rb.addNextLandMark(0., Color.RED);
		rb.addNextLandMark(1., Color.BLUE);

		//define a 2D painter of state values, specifying which attributes correspond 
		//to the x and y coordinates of the canvas
		StateValuePainter2D svp = new StateValuePainter2D(rb);
		svp.setXYAttByObjectClass(GridWorldDomain.CLASSAGENT, GridWorldDomain.ATTX,
				GridWorldDomain.CLASSAGENT, GridWorldDomain.ATTY);


		//create our ValueFunctionVisualizer that paints for all states
		//using the ValueFunction source and the state value painter we defined
		ValueFunctionVisualizerGUI gui = new ValueFunctionVisualizerGUI(
												allStates, svp, valueFunction);
        gui.setTitle(title);
		//define a policy painter that uses arrow glyphs for each of the grid world actions
		PolicyGlyphPainter2D spp = new PolicyGlyphPainter2D();
		spp.setXYAttByObjectClass(GridWorldDomain.CLASSAGENT, GridWorldDomain.ATTX,
				GridWorldDomain.CLASSAGENT, GridWorldDomain.ATTY);
		spp.setActionNameGlyphPainter(GridWorldDomain.ACTIONNORTH, new ArrowActionGlyph(0));
		spp.setActionNameGlyphPainter(GridWorldDomain.ACTIONSOUTH, new ArrowActionGlyph(1));
		spp.setActionNameGlyphPainter(GridWorldDomain.ACTIONEAST, new ArrowActionGlyph(2));
		spp.setActionNameGlyphPainter(GridWorldDomain.ACTIONWEST, new ArrowActionGlyph(3));
		spp.setRenderStyle(PolicyGlyphPainter2D.PolicyGlyphRenderStyle.DISTSCALED);


		//add our policy renderer to it
		gui.setSpp(spp);
		gui.setPolicy(p);

		//set the background color for places where states are not rendered to grey
		gui.setBgColor(Color.BLACK);

		//start it
		gui.initGUI();



	}


	public void experimentAndPlotter(){

		//different reward function for more interesting results
		((SimulatedEnvironment)env).setRf(new GoalBasedRF(this.goalCondition, 5.0, -0.1));

		/**
		 * Create factories for Q-learning agent and SARSA agent to compare
		 */
		LearningAgentFactory qLearningFactory = new LearningAgentFactory() {
			@Override
			public String getAgentName() {
				return "Q-Learning";
			}

			@Override
			public LearningAgent generateAgent() {
				return new QLearning(domain, 0.99, hashingFactory, 0.3, 0.1);
			}
		};

		LearningAgentFactory sarsaLearningFactory = new LearningAgentFactory() {
			@Override
			public String getAgentName() {
				return "SARSA";
			}

			@Override
			public LearningAgent generateAgent() {
				return new SarsaLam(domain, 0.99, hashingFactory, 0.0, 0.1, 1.);
			}
		};

		LearningAlgorithmExperimenter exp = new LearningAlgorithmExperimenter(env, 10, 100, 
												qLearningFactory, sarsaLearningFactory);
		exp.setUpPlottingConfiguration(500, 250, 2, 1000,
				TrialMode.MOSTRECENTANDAVERAGE,
				PerformanceMetric.CUMULATIVESTEPSPEREPISODE,
				PerformanceMetric.AVERAGEEPISODEREWARD);

		exp.startExperiment();
		exp.writeStepAndEpisodeDataToCSV("expData");

	}


	public static void main(String[] args) {

		AdvancedBehavior example = new AdvancedBehavior();
		String outputPath = "output/";

		//example.BFSExample(outputPath);
		//example.DFSExample(outputPath);
		//example.AStarExample(outputPath);
		//example.valueIterationExample(outputPath);
        //example.policyIterationExample(outputPath);
		//example.qLearningExample(outputPath);
		//example.sarsaLearningExample(outputPath);

		example.experimentAndPlotter();

		example.visualize(outputPath);

	}


}