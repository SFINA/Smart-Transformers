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
package power.smarttransformer;

import agents.backend.BackendParameterLoaderInterface;
import input.SfinaParameterLoader;
import java.io.File;
import java.io.FileNotFoundException;
import java.util.HashMap;
import java.util.Scanner;
import java.util.StringTokenizer;
import org.apache.log4j.Logger;
import power.backend.PowerFlowType;
//import power.backend1.PowerBackendParameter;

/**
 *
 * @author jose
 */
public class SmartTransformerLoader {
    
    private HashMap<Enum,Object> transformerParameters;
    
    private String columnSeparator;
    private static final Logger logger = Logger.getLogger(SmartTransformerLoader.class);
    
    public SmartTransformerLoader(String columnSeparator){
        this.columnSeparator=columnSeparator;
    }
    
    public HashMap<Enum,Object> loadSmartTransformerParameters(String location){
        HashMap<Enum,Object> transformerParameters = new HashMap();
        File file = new File(location);
        Scanner scr = null;
        try {
            scr = new Scanner(file);
            while(scr.hasNext()){
                StringTokenizer st = new StringTokenizer(scr.next(), columnSeparator);
                Enum param = null;
                Object value = null;
                switch(st.nextToken()){
                    case "smartTransformerAlgo":
                        param = SmartTransformerParameter.ALGORITHM;
                        value = Double.parseDouble(st.nextToken());
                        break;
                    case "position":
                        param = SmartTransformerParameter.LOCATION;
                        value = st.nextToken();
                        break;
                    default:
                        logger.debug("This Smart Transformer parameter is not supported or cannot be recognized");
                }
                transformerParameters.put(param, value);
            }
        }
        catch (FileNotFoundException ex){
            ex.printStackTrace();
        }
        return transformerParameters;
    }
    public void setSmartTransformerParameters(HashMap<Enum,Object> smartTransformerParameters){
        this.transformerParameters=smartTransformerParameters;
//        this.extractDomainParameters();
    }
}