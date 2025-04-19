classdef Sensibilities< handle
    %PARAMS Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        position (1,5) cell
        velocity (1,5) cell
        time (1,5) cell
        value (1,1) double   
    end
    
    methods

        function param = Sensibilities(Initvalue)
            param.value = Initvalue;
        end
        
        function savePosData(param,SimPosData,Dataset)
            param.position{Dataset} = SimPosData;
        end

        function saveVelData(param,SimVelData,Dataset)
            param.velocity{Dataset} = SimVelData;
        end

        function saveTime(param,SimTimeData,Dataset)
            param.time{Dataset} = SimTimeData;
        end

    end
end

