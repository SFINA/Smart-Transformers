/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package agent;

import java.util.ArrayList;
import java.util.List;
import java.util.HashMap;
import network.FlowNetwork;
import network.Link;
import org.apache.log4j.Logger;
import power.backend.PowerFlowType;
import power.backend.PowerBackendParameter;
import power.input.PowerLinkState;

import gurobi.*;

/**
 * Cascading failures mitigation strategy with Smart Transformers. 
 * 
 * @author jose
 */

public class PowerMitigationAgent extends PowerCascadeAgent {

    private static final Logger logger = Logger.getLogger(PowerMitigationAgent.class);
    private Double relRateChangePerEpoch;
    private int[] linkTransformer;
    private ArrayList<Link> PST;


    public PowerMitigationAgent(String experimentID,
            Double relRateChangePerEpoch,
            int[] linkTransformer) {
        super(experimentID,
                relRateChangePerEpoch);
        this.relRateChangePerEpoch = relRateChangePerEpoch;
        this.linkTransformer = linkTransformer;
    }
    
    /**
     * Implements the mitigation strategy.
     * 
     * @param flowNetwork 
     */
    @Override
    public void mitigateOverload(FlowNetwork flowNetwork){
        logger.debug("mitigation algorithm");
        PST = new ArrayList(); 
        
        PowerFlowType flowType = (PowerFlowType)getFlowDomainAgent().getDomainParameters().get(PowerBackendParameter.FLOW_TYPE);

        setSmartTransformer(flowNetwork);
        setFlowTypeParameter(PowerFlowType.DC);
        getControlAction(flowNetwork, PST);
        setFlowTypeParameter(flowType);
        pstAction (PST,flowNetwork);
        getFlowDomainAgent().flowAnalysis(flowNetwork);
        
    }
    
    /**
     * Sets the flow type parameter. AC or DC
     * @param flowType 
     */
    public void setFlowTypeParameter(PowerFlowType flowType) {
        this.getFlowDomainAgent().getDomainParameters().put(PowerBackendParameter.FLOW_TYPE, flowType);
    }

    /**
     * Placement of the Smart transformers.
     * @param flowNetwork 
     */
    private void setSmartTransformer(FlowNetwork flowNetwork){
        int l[] =this.linkTransformer;
        List<Integer> listPST = new ArrayList<>();
        for (int i=0;i<l.length;i++){
            listPST.add(l[i]);
        }
        
        for (Link link : flowNetwork.getLinks()){
            if ( listPST.contains( Integer.parseInt( link.getIndex() )) && link.isActivated() ){
//                logger.debug(" adding link");
                PST.add(link);
            }
        }   
    }
    
    /**
     * Calculates the initial flow and the control action.
     * @param flowNetwork
     * @param PST 
     */
    private void getControlAction (FlowNetwork flowNetwork, 
                                    ArrayList<Link> PST){
        double epsilon = 0.001;
        getFlowDomainAgent().flowAnalysis(flowNetwork); //(double)(flowNetwork.getLink(key).getProperty(PowerLinkState.POWER_FLOW_FROM_REAL))/flowNetwork.getLink(key).getCapacity()
        for (Link link :flowNetwork.getLinks()){
            link.addProperty(LinkTransformer.INITIAL_FLOW, (double)(link.getProperty(PowerLinkState.POWER_FLOW_FROM_REAL))/link.getCapacity());
        }
        System.out.println(" ");
        for (Link link : PST){
            link.addProperty(LinkTransformer.ANGLE,  link.getProperty(PowerLinkState.ANGLE_SHIFT));
            link.replacePropertyElement(PowerLinkState.ANGLE_SHIFT,(double) link.getProperty(LinkTransformer.ANGLE)+ 1.0); // change here for previous angle
//            logger.debug("changing angle");
            getFlowDomainAgent().flowAnalysis(flowNetwork);
            HashMap<String,Double> auxMap = new HashMap<>(); 
            for (Link link1:flowNetwork.getLinks()){
                double action = ((double)(link1.getProperty(PowerLinkState.POWER_FLOW_FROM_REAL))/(link1.getCapacity()))- (double)link1.getProperty(LinkTransformer.INITIAL_FLOW);
                if (action < -epsilon || epsilon< action){
                    auxMap.put(link1.getIndex(),action/1.0);
                }
                else{
                    auxMap.put(link1.getIndex(),null);
                }
            }
            link.addProperty(LinkTransformer.CONTROL_ACTION, auxMap);
            link.replacePropertyElement(PowerLinkState.ANGLE_SHIFT,(double) link.getProperty(LinkTransformer.ANGLE));
        }   
    }
    
    /**
     * Formulates and solves the optimization problem.
     * Two mitigation strategies are implemented. 
     * Gurobi API and solver are used to solve the QP and MILP problems.
     * 
     * @param PST
     * @param flowNetwork 
     */
    private void pstAction (ArrayList<Link> PST,
            FlowNetwork flowNetwork){
        try{
            double M = 30.0;
            double epsilon = 0.001;
            GRBEnv    env   = new GRBEnv("");
            env.set(GRB.IntParam.OutputFlag, 0);
            GRBModel  model = new GRBModel(env);
            
            
            HashMap<String,GRBVar> xl = new HashMap<>();
            HashMap<String,GRBVar> anglel = new HashMap<>();

            for(Link link: flowNetwork.getLinks()){
                xl.put(link.getIndex(),model.addVar(-M, M, 0.0, GRB.CONTINUOUS, "xi" + link.getIndex()));
            }
            model.update();


            for (Link link: PST){
                anglel.put(link.getIndex(), model.addVar(-7.0, 7.0, 0.0, GRB.CONTINUOUS, "angle" + link.getIndex()));
            }
            
            model.update();
            
            for (Link link: PST){
                GRBLinExpr chs = new GRBLinExpr();
                GRBLinExpr rhs = new GRBLinExpr();
                GRBLinExpr lhs = new GRBLinExpr();
                chs.addTerm(1.0, anglel.get(link.getIndex()));
                chs.addConstant((double) link.getProperty(LinkTransformer.ANGLE));
                rhs.addConstant(16.0);
                lhs.addConstant(-16.0);
                model.addConstr(lhs, GRB.LESS_EQUAL, chs, "");
                model.addConstr(chs, GRB.LESS_EQUAL, rhs, "");
            }
            model.update();
            
            for (Link link: flowNetwork.getLinks()){
                GRBLinExpr lhs = new GRBLinExpr();
                GRBLinExpr rhs = new GRBLinExpr();
                lhs.addTerm(1.0, xl.get(link.getIndex()));
                rhs.addConstant((double) link.getProperty(LinkTransformer.INITIAL_FLOW)); //initialFlow.get(key)
                HashMap<String,Double> auxMap = new HashMap<>();
                for (Link link1: PST){
                    auxMap=(HashMap<String,Double>)link1.getProperty(LinkTransformer.CONTROL_ACTION);
                    if (auxMap.get(link.getIndex())!=null){
                        rhs.addTerm(auxMap.get(link.getIndex()), anglel.get(link1.getIndex()));
                    }
                }
                model.addConstr(lhs, GRB.EQUAL, rhs, "");
            }
            model.update();
            
            if (true){ 
                HashMap<String,GRBVar> z_pos = new HashMap<>();
                HashMap<String,GRBVar> z_neg = new HashMap<>();
                double beta[] = new double[]{0.60,0.8,0.9};
                double beta_cost[] = new double[]{1.0,10.0,100.0,1000.0};
                for (int j = 0;j < beta.length;j++){
                    for(Link link: flowNetwork.getLinks()){
                        z_pos.put(link.getIndex()+Integer.toString(j),model.addVar(0.0, 1.0, 0.0, GRB.BINARY, "z_pos" + link.getIndex()));
                        z_neg.put(link.getIndex()+Integer.toString(j),model.addVar(0.0, 1.0, 0.0, GRB.BINARY, "z_neg" + link.getIndex()));

                    }
                }
                model.update();
                
                for (int j =0;j < beta.length;j++){
                    for(Link link: flowNetwork.getLinks()){
                        GRBLinExpr lhs = new GRBLinExpr();
                        GRBLinExpr rhs_1 = new GRBLinExpr();
                        GRBLinExpr rhs_2 = new GRBLinExpr();
                        
                        
                        lhs.addTerm(1.0,xl.get(link.getIndex()));
                        lhs.addConstant(-beta[j]);
                        rhs_1.addTerm(M, z_pos.get(link.getIndex()+Integer.toString(j)));
                        rhs_2.addTerm(M, z_pos.get(link.getIndex()+Integer.toString(j)));
                        rhs_2.addConstant(-M);
                        rhs_2.addConstant(epsilon);
                        model.addConstr(lhs, GRB.LESS_EQUAL, rhs_1, "");
                        model.addConstr(lhs, GRB.GREATER_EQUAL, rhs_2, "");
                    }
                    model.update();
                    for(Link link: flowNetwork.getLinks()){
                        GRBLinExpr lhs = new GRBLinExpr();
                        GRBLinExpr rhs_1 = new GRBLinExpr();
                        GRBLinExpr rhs_2 = new GRBLinExpr();
                        
                        
                        lhs.addTerm(-1.0,xl.get(link.getIndex()));
                        lhs.addConstant(-beta[j]);
                        rhs_1.addTerm(M, z_neg.get(link.getIndex()+Integer.toString(j)));
                        rhs_2.addTerm(M, z_neg.get(link.getIndex()+Integer.toString(j)));
                        rhs_2.addConstant(-M);
                        rhs_2.addConstant(epsilon);
                        model.addConstr(lhs, GRB.LESS_EQUAL, rhs_1, "");
                        model.addConstr(lhs, GRB.GREATER_EQUAL, rhs_2, "");
                    }
                    model.update();
                }
                GRBQuadExpr obj = new GRBQuadExpr();
                for (int j =0;j < beta.length;j++){
                    for (String key : xl.keySet()){
                        obj.addTerm(beta_cost[j], z_pos.get(key+Integer.toString(j)));
                        obj.addTerm(beta_cost[j], z_neg.get(key+Integer.toString(j)));
                    }
                }
                for (Link link :PST){
                    obj.addTerm(beta_cost[3], z_pos.get(link.getIndex()+Integer.toString(2)));
                    obj.addTerm(beta_cost[3], z_neg.get(link.getIndex()+Integer.toString(2)));
                }
                
                for (String key : anglel.keySet()){
                    obj.addTerm(1.0, anglel.get(key),anglel.get(key));
                }
                model.setObjective(obj);
                model.update();

                model.optimize();
                
            }
            else{
                GRBQuadExpr obj = new GRBQuadExpr();
                for (String key : xl.keySet()){
                    obj.addTerm(1.0, xl.get(key),xl.get(key));
                }
                model.setObjective(obj);
                model.update();

                model.optimize();
            }
            
//            for (Link link: PST){
//                link.replacePropertyElement(PowerLinkState.ANGLE_SHIFT,(double) link.getProperty(LinkTransformer.ANGLE) + anglel.get(link.getIndex()).get(GRB.DoubleAttr.X)); // sum previous angle
//                logger.debug(link.getIndex()+" "+ link.getProperty(PowerLinkState.ANGLE_SHIFT));
//            }
            getFlowDomainAgent().flowAnalysis(flowNetwork);
//            for (String key: xl.keySet()){
//                logger.debug(key +" " +  xl.get(key).get(GRB.DoubleAttr.X)+ " real "+ 
//                        (double)(flowNetwork.getLink(key).getProperty(PowerLinkState.POWER_FLOW_FROM_REAL))/flowNetwork.getLink(key).getCapacity());
//                
//            }
            
            model.dispose();
            env.dispose();
            
            
        }
        catch (GRBException e) {
            System.out.println("Error code: " + e.getErrorCode() + ". " +
                               e.getMessage());
        }
        
    }
    
  
}

 