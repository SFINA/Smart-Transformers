/*
 * Copyright (C) 2016 SFINA Team
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 */
package agent;

import power.smarttransformer.LinkTransformer;
import java.util.ArrayList;
import java.util.List;
import java.util.HashMap;
import network.FlowNetwork;
import network.Link;
import org.apache.log4j.Logger;
import power.backend.PowerFlowType;
import power.backend.PowerBackendParameter;
import power.input.PowerLinkState;

import power.smarttransformer.SmartTransformerParameter;

import gurobi.*;
import power.smarttransformer.SmartTransformerLoader;

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
    private Double PST_Algo;
    public HashMap<Enum,Object> transformerParameters;
    private static String experimentID;


    public PowerMitigationAgent(String experimentID,
            Double relRateChangePerEpoch,
            int[] linkTransformer) {
        super(experimentID,
                relRateChangePerEpoch);
        this.relRateChangePerEpoch = relRateChangePerEpoch;
        this.linkTransformer = linkTransformer;
        this.experimentID=experimentID;
    }
    
    /**
     * Implements the mitigation strategy.
     * 
     * @param flowNetwork 
     */
    @Override
    public void mitigateOverload(FlowNetwork flowNetwork){
        
        SmartTransformerLoader smartTransformerLoader= new SmartTransformerLoader(getFlowDomainAgent().getParameterColumnSeparator());
        transformerParameters=smartTransformerLoader.loadSmartTransformerParameters("experiments/"+experimentID+"/peer-0/input/transformerApplication.txt"); 
        PST_Algo = (Double) transformerParameters.get(SmartTransformerParameter.ALGORITHM);
        
        logger.debug("mitigation algorithm");
        PST = new ArrayList(); 
        
        PowerFlowType flowType = (PowerFlowType)getFlowDomainAgent().getDomainParameters().get(PowerBackendParameter.FLOW_TYPE);

        if (PST_Algo != 0.0){
            setSmartTransformer(flowNetwork);
            setFlowTypeParameter(PowerFlowType.DC);
            getControlAction(flowNetwork, PST);
            setFlowTypeParameter(flowType);
            pstAction (PST,flowNetwork);
            getFlowDomainAgent().flowAnalysis(flowNetwork);
        }
        
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
        String[] l  = transformerParameters.get(SmartTransformerParameter.LOCATION).toString().split(",");
        List<Integer> listPST = new ArrayList<>();
        for (int i=0;i<l.length;i++){
            listPST.add(Integer.parseInt(l[i]));
        }
        
        for (Link link : flowNetwork.getLinks()){
            if ( listPST.contains( Integer.parseInt( link.getIndex() )) && link.isActivated() ){
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
        getFlowDomainAgent().flowAnalysis(flowNetwork); 
        for (Link link :flowNetwork.getLinks()){
            link.addProperty(LinkTransformer.INITIAL_FLOW, (double)(link.getProperty(PowerLinkState.POWER_FLOW_FROM_REAL))/link.getCapacity());
        }
        for (Link link : PST){
            link.addProperty(LinkTransformer.ANGLE,  link.getProperty(PowerLinkState.ANGLE_SHIFT));
            link.replacePropertyElement(PowerLinkState.ANGLE_SHIFT,(double) link.getProperty(LinkTransformer.ANGLE)+ 1.0); 
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
            
            if (PST_Algo==1.0){ 
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
            
            
            model.dispose();
            env.dispose();
            
            
        }
        catch (GRBException e) {
            System.out.println("Error code: " + e.getErrorCode() + ". " +
                               e.getMessage());
        }
        
    }
    
  
}

 