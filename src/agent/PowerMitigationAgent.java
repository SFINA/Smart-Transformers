/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package agent;

/**
 *
 * @author jose
 */

import agent.PowerCascadeAgent;
import event.Event;
import event.EventType;
import event.NetworkComponent;
import java.util.ArrayList;
import java.util.List;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import network.FlowNetwork;
import network.Link;
import network.LinkState;
import network.Node;
import org.apache.log4j.Logger;
import power.backend.PowerFlowType;
import power.input.PowerNodeType;
import power.backend.PowerBackendParameter;
import power.input.PowerLinkState;
import power.input.PowerNodeState;
import protopeer.measurement.MeasurementFileDumper;
import protopeer.measurement.MeasurementLog;
import protopeer.measurement.MeasurementLoggerListener;
import protopeer.util.quantities.Time;

import agent.LinkTransformer;


import gurobi.*;

public class PowerMitigationAgent extends PowerCascadeAgent {

    private static final Logger logger = Logger.getLogger(PowerMitigationAgent.class);
    private Double relRateChangePerEpoch;
    private ArrayList<Node> generators; 
    private ArrayList<Link> PST;
    private HashMap<String,HashMap<String,Double>> controlAction;
    private HashMap<String,Double> initialFlow;
    private HashMap<String,Double> actualAngle;
    //private ArrayList<String> PST1;
    private Node slack;

    public PowerMitigationAgent(String experimentID,
            Double relRateChangePerEpoch) {
        super(experimentID,
                relRateChangePerEpoch);
        this.relRateChangePerEpoch = relRateChangePerEpoch;
    }
    

    @Override
    public void mitigateOverload(FlowNetwork flowNetwork){
        logger.debug("mitigation algorithm");
        PST = new ArrayList(); 
        initialFlow = new HashMap<>();
        controlAction = new HashMap<>();
//        actualAngle = new HashMap<>();
        
        PowerFlowType flowType = (PowerFlowType)getFlowDomainAgent().getDomainParameters().get(PowerBackendParameter.FLOW_TYPE);

        
        setSmartTransformer(flowNetwork);
        setFlowTypeParameter(PowerFlowType.DC);
        getControlAction(flowNetwork, PST);
        setFlowTypeParameter(flowType);
        pstAction (PST,flowNetwork);
        //setFlowTypeParameter(flowType);
        getFlowDomainAgent().flowAnalysis(flowNetwork);
        
    }
    
    public void setFlowTypeParameter(PowerFlowType flowType) {
        this.getFlowDomainAgent().getDomainParameters().put(PowerBackendParameter.FLOW_TYPE, flowType);
    }

    private void setSmartTransformer(FlowNetwork flowNetwork){
//        int l[] = new int[] {30,22,24};
        int l[] = new int[] {30, 3, 12, 44, 6};
//        int l[] = new int[] {};
        List<Integer> listPST = new ArrayList<>();
        for (int i=0;i<l.length;i++){
            listPST.add(l[i]);
        }
        
//        listPST.add(30);
//        listPST.add(22);//21
//        listPST.add(24);//25
//        int listPST[] = new int[]{30,22,24};
        //listPST.add(26);
        for (Link link : flowNetwork.getLinks()){
            //logger.debug(link.getIndex());
            if ( listPST.contains( Integer.parseInt( link.getIndex() )) && link.isActivated() ){
                logger.debug(" adding link");
                PST.add(link);
            }
        }   
    }
    
    private void getControlAction (FlowNetwork flowNetwork, 
                                    ArrayList<Link> PST){
        //HashMap<String,Double> initialFlow = new HashMap<String,Double>();
        // HashMap<String,HashMap<String,Double>> controlAction = new HashMap<>();
        // double actualAngle;
        double epsilon = 0.001;
        getFlowDomainAgent().flowAnalysis(flowNetwork); //(double)(flowNetwork.getLink(key).getProperty(PowerLinkState.POWER_FLOW_FROM_REAL))/flowNetwork.getLink(key).getCapacity()
        for (Link link :flowNetwork.getLinks()){
            link.addProperty(LinkTransformer.INITIAL_FLOW, (double)(link.getProperty(PowerLinkState.POWER_FLOW_FROM_REAL))/link.getCapacity());
            initialFlow.put(link.getIndex(), (double)(link.getProperty(PowerLinkState.POWER_FLOW_FROM_REAL))/link.getCapacity()); ///link.getCapacity()
        }
        System.out.println(" ");
        for (Link link : PST){
            link.addProperty(LinkTransformer.ANGLE,  link.getProperty(PowerLinkState.ANGLE_SHIFT));
//            actualAngle.put(link.getIndex(),(double) link.getProperty(PowerLinkState.ANGLE_SHIFT));
            //link.replacePropertyElement(PowerLinkState.ANGLE_SHIFT,actualAngle.get(link.getIndex())+ 1.0); // change here for previous angle
            link.replacePropertyElement(PowerLinkState.ANGLE_SHIFT,(double) link.getProperty(LinkTransformer.ANGLE)+ 1.0); // change here for previous angle
            logger.debug("changing angle");
            getFlowDomainAgent().flowAnalysis(flowNetwork);
            HashMap<String,Double> auxMap = new HashMap<>(); 
            for (Link link1:flowNetwork.getLinks()){
                double action = ((double)(link1.getProperty(PowerLinkState.POWER_FLOW_FROM_REAL))/(link1.getCapacity()))- (double)link1.getProperty(LinkTransformer.INITIAL_FLOW);///(link1.getCapacity())
                //logger.debug(action);
                if (action < -epsilon || epsilon< action){
                    //logger.debug("not null");
                    auxMap.put(link1.getIndex(),action/1.0);
                }
                else{
                    //logger.debug("null " + link1.getIndex() + " " + action);
                    auxMap.put(link1.getIndex(),null);
                }
                //controlAction.put( link.getIndex(), new HashMap(){{put(link1.getIndex(),(link1.getFlow()- initialFlow.get(link1.getIndex()))/(link1.getCapacity() ) );}}  );
                //logger.debug(String.valueOf( link1.getFlow() - initialFlow.get(link1.getIndex())   )); //link1.getFlow()
                //logger.debug(String.valueOf(controlAction.get(link.getIndex()).get(link1.getIndex())    ));
            }
            link.addProperty(LinkTransformer.CONTROL_ACTION, auxMap);
            controlAction.put( link.getIndex(),auxMap);
            //link.replacePropertyElement(PowerLinkState.ANGLE_SHIFT,actualAngle.get(link.getIndex()));
            link.replacePropertyElement(PowerLinkState.ANGLE_SHIFT,(double) link.getProperty(LinkTransformer.ANGLE));
        }   
    }
    
    private void pstAction (ArrayList<Link> PST,
            FlowNetwork flowNetwork){
        //logger.debug(String.valueOf(initialFlow.get("0"))); 
        try{
            double M = 30.0;
            double epsilon = 0.001;
            GRBEnv    env   = new GRBEnv("");
            env.set(GRB.IntParam.OutputFlag, 0);
            GRBModel  model = new GRBModel(env);
            
            
            HashMap<String,GRBVar> xl = new HashMap<>();
            HashMap<String,GRBVar> anglel = new HashMap<>();
            
            
//            for(String key : initialFlow.keySet()){
//                xl.put(key,model.addVar(-M, M, 0.0, GRB.CONTINUOUS, "xi" + key));
//            }
            for(Link link: flowNetwork.getLinks()){
                xl.put(link.getIndex(),model.addVar(-M, M, 0.0, GRB.CONTINUOUS, "xi" + link.getIndex()));
            }
            model.update();

//            for (String key : controlAction.keySet()){
//                anglel.put(key, model.addVar(-7.0, 7.0, 0.0, GRB.CONTINUOUS, "angle" + key));
//            }

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
                //logger.debug(initialFlow.get(key));
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
            
            if (true){ // change condition, is temporal
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
                
                //for (String key: xl.keySet()){
                    //logger.debug(key +" " +  xl.get(key).get(GRB.DoubleAttr.X));
                    //logger.debug(z_pos.get(key).get(GRB.DoubleAttr.X));
                    //logger.debug(z_neg.get(key).get(GRB.DoubleAttr.X));
                //}
                
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
            
            for (Link link: PST){
                link.replacePropertyElement(PowerLinkState.ANGLE_SHIFT,(double) link.getProperty(LinkTransformer.ANGLE) + anglel.get(link.getIndex()).get(GRB.DoubleAttr.X)); // sum previous angle
                logger.debug(link.getIndex()+" "+ link.getProperty(PowerLinkState.ANGLE_SHIFT));
            }
            getFlowDomainAgent().flowAnalysis(flowNetwork);
            //for (String key: anglel.keySet()){
            //    logger.debug(anglel.get(key).get(GRB.DoubleAttr.X));
            //}
            for (String key: xl.keySet()){
                logger.debug(key +" " +  xl.get(key).get(GRB.DoubleAttr.X)+ " real "+ 
                        (double)(flowNetwork.getLink(key).getProperty(PowerLinkState.POWER_FLOW_FROM_REAL))/flowNetwork.getLink(key).getCapacity());
                
            }
            
            model.dispose();
            env.dispose();
            
            
        }
        catch (GRBException e) {
            System.out.println("Error code: " + e.getErrorCode() + ". " +
                               e.getMessage());
        }
        
    }
    
  
}

 