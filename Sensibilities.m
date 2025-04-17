classdef Sensibilities< handle
    %PARAMS Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        position (1,5) cell
        velocity (1,5) cell
        value (1,1) double
        time (1,1) cell
    end
    
    methods

        function param = Sensibilities(Initvalue)
            param.value = Initvalue;
        end
        
        function savePosData(param,SimPosData,Dataset)
            param.position(Dataset) = SimPosData;
        end

        function saveVelData(param,SimVelData,Dataset)
            param.velocity(Dataset) = SimVelData;
        end

    end
end

