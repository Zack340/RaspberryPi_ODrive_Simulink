classdef ODriveRasPi < realtime.internal.SourceSampleTime ...
        & coder.ExternalDependency ...
        & matlab.system.mixin.Propagates ...
        & matlab.system.mixin.CustomIcon
    
    %
    %	ODrive Driver for Raspberry Pi
    %
    %	Author : Eisuke Matsuzaki
    %	Created on : 08/03/2020
    %	Copyright (c) 2020 d’Arbeloff Lab, MIT Department of Mechanical Engineering
    %	Released under the GNU license
    %
    
    %#codegen
    %#ok<*EMCA>
    
    properties
        % Public, tunable properties.
    end
    
    properties (Nontunable, Logical)
        isPort = false; % Enable specify serial port
        isAxis0 = true; % Enable Axis 0 control
        isAxis1 = false; % Enable Axis 1 control
        isInCpr0 = false; % Enable circular position control
        isInCpr1 = false; % Enable circular position control
        isVelRamp0 = false; % Enable velocity ramp control
        isVelRamp1 = false; % Enable velocity ramp control
        isExternal0 = false; % Enable external inputs
        isExternal1 = false; % Enable external inputs
    end
    
    properties (Nontunable)
        serial = '123456789ABC'; % ODrive serial number
        portName = '/dev/ttyACM0'; % ODrive connection port
        ctrlMode0 = 'Position'; % Control mode
        ctrlMode1 = 'Position'; % Control mode
        watchdog0 = 0; % Watchdog timeout [s]
        watchdog1 = 0; % Watchdog timeout [s]
        posGain0 = 20.0; % Position P gain [(counts/s) / counts]
        posGain1 = 20.0; % Position P gain [(counts/s) / counts]
        velGain0 = 0.0005; % Velocity P gain [A/(counts/s)]
        velGain1 = 0.0005; % Velocity P gain [A/(counts/s)]
        velIGain0 = 0.001; % Velocity I gain [A/((counts/s) * s)]
        velIGain1 = 0.001; % Velocity I gain [A/((counts/s) * s)]
        velLimit0 = 20000.0; % Velocity limit [counts/s]
        velLimit1 = 20000.0; % Velocity limit [counts/s]
        velLimitTol0 = 1.2; % Velocity limit tolerance
        velLimitTol1 = 1.2; % Velocity limit tolerance
        velRampRate0 = 10000.0; % Velocity ramp rate [(counts/s) / s]
        velRampRate1 = 10000.0; % Velocity ramp rate [(counts/s) / s]
    end
    
    properties (Access = private)
        ctrlMode = uint16([3, 3]);
        ctrlModeName = {'Voltage', 'Current', 'Velocity', 'Position', 'Trajectory'};
    end
    
    properties(Constant, Hidden)
        ctrlMode0Set = matlab.system.StringSet({'Position', 'Velocity', 'Current'});
        ctrlMode1Set = matlab.system.StringSet({'Position', 'Velocity', 'Current'});
        inputName = {'M0 position ref', 'M0 velocity ref', 'M0 current ref',...
                     'M0 pos gain', 'M0 vel gain', 'M0 vel I gain', 'M0 vel integral ref', 'M0 vel int trigger',...
                     'M0 vel ramp enable', 'M0 vel ramp', 'M0 vel ramp rate',...
                     'M0 vel limit', 'M0 vel limit tol',...
                     'M1 position ref', 'M1 velocity ref', 'M1 current ref',...
                     'M1 pos gain', 'M1 vel gain', 'M1 vel I gain', 'M1 vel integral ref', 'M1 vel int trigger',...
                     'M1 vel ramp enable', 'M1 vel ramp', 'M1 vel ramp rate',...
                     'M1 vel limit', 'M1 vel limit tol'};
        inputType = {'double', 'double', 'double',...
                     'double', 'double', 'double', 'double', 'logical',...
                     'logical', 'double', 'double', 'double', 'double',...
                     'double', 'double', 'double',...
                     'double', 'double', 'double', 'double', 'logical',...
                     'logical', 'double', 'double', 'double', 'double'};
        outputName = {'Axis error',...
                      'M0 position act', 'M0 velocity act', 'M0 current act', 'M0 vel integral act',...
                      'M1 position act', 'M1 velocity act', 'M1 current act', 'M1 vel integral act'};
        outputType = {'int16', 'double', 'double', 'double', 'double', 'double', 'double', 'double', 'double'};
        outputSize = {2, 1, 1, 1, 1, 1, 1, 1, 1};
    end
    
    methods
        % Constructor
        function obj = ODriveRasPi(varargin)
            % Support name-value pair arguments when constructing the object.
            setProperties(obj,nargin,varargin{:});
        end
    end
    
    methods (Access=protected)
        function setupImpl(obj) %#ok<MANU>
            if isempty(coder.target)
                % Place simulation setup code here
            else
                coder.cinclude('odrive_raspi.h');
                
                for i = 1:length(obj.ctrlModeName)
                    if(strcmp(obj.ctrlMode0, obj.ctrlModeName{i}))
                        obj.ctrlMode(1) = uint8(i-1);
                    end
                    if(strcmp(obj.ctrlMode1, obj.ctrlModeName{i}))
                        obj.ctrlMode(2) = uint8(i-1);
                    end
                end
                
                settings = struct('isPort', obj.isPort,...
                                  'serial', uint8(obj.serial),...
                                  'portName', uint8(obj.portName),...
                                  'isAxis', [obj.isAxis0, obj.isAxis1],...
                                  'isExternal', [obj.isExternal0, obj.isExternal1],...
                                  'controlMode', obj.ctrlMode,...
                                  'posGain', single([obj.posGain0, obj.posGain1]),...
                                  'velGain', single([obj.velGain0, obj.velGain1]),...
                                  'velIntegratorGain', single([obj.velIGain0, obj.velIGain1]),...
                                  'velLimit', single([obj.velLimit0, obj.velLimit1]),...
                                  'velLimitTolerance', single([obj.velLimitTol0, obj.velLimitTol1]),...
                                  'velRampRate', single([obj.velRampRate0, obj.velRampRate1]),...
                                  'setPointsInCpr', [obj.isInCpr0, obj.isInCpr1],...
                                  'velRampEnable', [obj.isVelRamp0, obj.isVelRamp1],...
                                  'watchdogTimeout', single([obj.watchdog0, obj.watchdog1]));
                
                coder.cstructname(settings, 'struct Settings', 'extern', 'HeaderFile', 'odrive_raspi.h');
                coder.ceval('initialize', coder.ref(settings));
            end
        end
        
        function varargout = stepImpl(obj, varargin)   %#ok<MANU>
            y = {int16([0; 0]), 0, 0, 0, 0, 0, 0, 0, 0};
            if isempty(coder.target)
                indexNum = getOutputNumIndex(obj);
                for i=1:length(indexNum)
                    varargout{i} = y{indexNum(i)};
                end
            else
                u = zeros(1, length(obj.inputName));
                indexNum = getInputNumIndex(obj);
                for i=1:length(indexNum)
                    u(indexNum(i)) = varargin{i};
                end
                
                data = struct('error', zeros(2, 1, 'int16'),...
                              'posSetpoint', single([u(1), u(14)]),...
                              'velSetpoint', single([u(2), u(15)]),...
                              'currentSetpoint', single([u(3), u(16)]),...
                              'posGain', single([u(4), u(17)]),...
                              'velGain', single([u(5), u(18)]),...
                              'velIntegratorGain', single([u(6), u(19)]),...
                              'velIntegratorCurrentRef', single([u(7), u(20)]),...
                              'velIntegratorCurrentTrigger', logical([u(8), u(21)]),...
                              'velRampEnable', logical([u(9), u(22)]),...
                              'velRampTarget', single([u(10), u(23)]),...
                              'velRampRate', single([u(11), u(24)]),...
                              'velLimit', single([u(12), u(25)]),...
                              'velLimitTolerance', single([u(13), u(26)]),...
                              'actualPosition', zeros(2, 1, 'single'),...
                              'actualVelocity', zeros(2, 1, 'single'),...
                              'actualCurrent', zeros(2, 1, 'single'),...
                              'velIntegratorCurrentAct', zeros(2, 1, 'single'));
                coder.cstructname(data, 'struct Data', 'extern', 'HeaderFile', 'odrive_raspi.h');
                coder.ceval('step', coder.ref(data));
                
                y{1} = data.error;
                y{2} = double(data.actualPosition(1));
                y{3} = double(data.actualVelocity(1));
                y{4} = double(data.actualCurrent(1));
                y{5} = double(data.velIntegratorCurrentAct(1));
                y{6} = double(data.actualPosition(2));
                y{7} = double(data.actualVelocity(2));
                y{8} = double(data.actualCurrent(2));
                y{9} = double(data.velIntegratorCurrentAct(2));
                
                indexNum = getOutputNumIndex(obj);
                for i=1:length(indexNum)
                    varargout{i} = y{indexNum(i)};
                end
            end
        end
        
        function releaseImpl(obj) %#ok<MANU>
            if isempty(coder.target)
                % Place simulation termination code here
            else
                coder.ceval('terminate');
            end
        end
    end
    
    methods (Access=protected)
        %% Define output properties
        function num = getNumInputsImpl(obj)
            num = sum(getInputIndex(obj));
        end
        
        function num = getNumOutputsImpl(obj)
            num = sum(getOutputIndex(obj));
        end
        
        function flag = isInputSizeLockedImpl(~,~)
            flag = true;
        end
        
        function flag = isOutputSizeLockedImpl(~,~)
            flag = true;
        end
        
        function varargout = isInputFixedSizeImpl(obj,~)
            for i = 1:sum(getInputIndex(obj))
                varargout{i} = true;
            end
        end
        
        function varargout = isOutputFixedSizeImpl(obj,~)
            for i = 1:sum(getOutputIndex(obj))
                varargout{i} = true;
            end
        end
        
        function flag = isInputComplexityLockedImpl(~,~)
            flag = true;
        end
        
        function flag = isOutputComplexityLockedImpl(~,~)
            flag = true;
        end
        
        function varargout = isInputComplexImpl(obj)
            for i = 1:sum(getInputIndex(obj))
                varargout{i} = false;
            end
        end
        
        function varargout = isOutputComplexImpl(obj)
            for i = 1:sum(getOutputIndex(obj))
                varargout{i} = false;
            end
        end
        
        function varargout = getInputSizeImpl(obj)
            for i = 1:sum(getInputIndex(obj))
                varargout{i} = [1, 1];
            end
        end
        
        function varargout = getOutputSizeImpl(obj)
            index = getOutputIndex(obj);
            j = 1;
            for i = 1:length(index)
                if index(i)
                    varargout{j} = obj.outputSize{i};
                    j = j + 1;
                end
            end
        end
        
        function varargout = getInputDataTypeImpl(obj)
            index = getinputIndex(obj);
            j = 1;
            for i = 1:length(obj.inputType)
                if index(i)
                    varargout{j} = obj.inputType{i};
                    j = j + 1;
                end
            end
        end
        
        function varargout = getOutputDataTypeImpl(obj)
            index = getOutputIndex(obj);
            j = 1;
            for i = 1:length(index)
                if index(i)
                    varargout{j} = obj.outputType{i};
                    j = j + 1;
                end
            end
        end
        
        function varargout = getInputNamesImpl(obj)
            index = getInputIndex(obj);
            j = 1;
            for i = 1:length(obj.inputName)
                if index(i)
                    varargout{j} = obj.inputName{i};
                    j = j + 1;
                end
            end
        end
        
        function varargout = getOutputNamesImpl(obj)
            index = getOutputIndex(obj);
            j = 1;
            for i = 1:length(index)
                if index(i)
                    varargout{j} = obj.outputName{i};
                    j = j + 1;
                end
            end
        end
        
        function icon = getIconImpl(obj)
            if obj.isPort
                text1 = ['Port: ', obj.portName];
            else
                text1 = ['Serial: ', obj.serial];
            end
            if obj.isAxis0
                text2 = ['M0: ', obj.ctrlMode0, ' control'];
            else
                text2 = 'M0: Disable';
            end
            if obj.isAxis1
                text3 = ['M1: ', obj.ctrlMode1, ' control'];
            else
                text3 = 'M1: Disable';
            end
            icon = {'ODrive', '', text1, '', text2, text3};
        end
        
        function index = getInputIndex(obj)
            isPos0 = strcmp(obj.ctrlMode0, 'Position');
            isCur0 = ~strcmp(obj.ctrlMode0, 'Current');
            isPos1 = strcmp(obj.ctrlMode1, 'Position');
            isCur1 = ~strcmp(obj.ctrlMode1, 'Current');
            index = [obj.isAxis0 &...
                    [isPos0, isCur0, true,...
                     obj.isExternal0 & ...
                    [isPos0, isCur0, isCur0, isCur0, isCur0,...
                     isCur0 & [obj.isVelRamp0, obj.isVelRamp0, obj.isVelRamp0],...
                     isCur0, isCur0]],...
                     obj.isAxis1 &...
                    [isPos1, isCur1, true,...
                     obj.isExternal1 & ...
                    [isPos1, isCur1, isCur1, isCur1, isCur1,...
                     isCur1 & [obj.isVelRamp1, obj.isVelRamp1, obj.isVelRamp1],...
                     isCur1, isCur1]]];
        end
        
        function indexNum = getInputNumIndex(obj)
            index = getInputIndex(obj);
            indexNum = zeros(1, sum(index));
            j = 1;
            for i=1:length(index)
               if index(i)
                   indexNum(j) = i;
                   j = j + 1;
               end
            end
        end
        
        function index = getOutputIndex(obj)
            index = [true,...
                     obj.isAxis0, obj.isAxis0, obj.isAxis0, obj.isAxis0,...
                     obj.isAxis1, obj.isAxis1, obj.isAxis1, obj.isAxis1];
        end
        
        function indexNum = getOutputNumIndex(obj)
            index = getOutputIndex(obj);
            indexNum = zeros(1, sum(index));
            j = 1;
            for i=1:length(index)
               if index(i)
                   indexNum(j) = i;
                   j = j + 1;
               end
            end
        end
    end
    
    methods (Static, Access=protected)
        function simMode = getSimulateUsingImpl(~)
            simMode = 'Interpreted execution';
        end
        
        function isVisible = showSimulateUsingImpl
            isVisible = false;
        end
        
        function groups = getPropertyGroupsImpl()
            configGroup = matlab.system.display.Section(...
                'Title', 'General configuration', 'PropertyList', {'SampleTime', 'isPort', 'portName', 'serial'});
            configGroup.Actions = matlab.system.display.Action(...
                @(~,obj)checkOdriveSerialNumber(obj), 'Label', '  Detect ODrive Serial Number  ',...
                'Alignment', 'right');
            axis0Group1 = matlab.system.display.Section(...
                'Title', 'Axis', 'PropertyList', {'isAxis0', 'ctrlMode0', 'isInCpr0', 'isVelRamp0', 'watchdog0'});
            axis0Group2 = matlab.system.display.Section(...
                'Title', 'Controller config', 'PropertyList', {'isExternal0', 'posGain0',...
                                                               'velGain0', 'velIGain0',...
                                                               'velLimit0', 'velLimitTol0', 'velRampRate0'});
            axis0Group2.Actions = matlab.system.display.Action(...
                @(~,obj)getOdriveAxis0Configs(obj), 'Label', '  Load ODrive configuration  ',...
                'Alignment', 'right');
            axis0Group = matlab.system.display.SectionGroup(...
                'Title', 'Axis 0', 'Sections', [axis0Group1, axis0Group2]);
            axis1Group1 = matlab.system.display.Section(...
                'Title', 'Axis', 'PropertyList', {'isAxis1', 'ctrlMode1', 'isInCpr1', 'isVelRamp1', 'watchdog1'});
            axis1Group2 = matlab.system.display.Section(...
                'Title', 'Controller config', 'PropertyList', {'isExternal1', 'posGain1',...
                                                               'velGain1', 'velIGain1',...
                                                               'velLimit1', 'velLimitTol1', 'velRampRate1'});
            axis1Group2.Actions = matlab.system.display.Action(...
                @(~,obj)getOdriveAxis1Configs(obj), 'Label', '  Load ODrive configuration  ',...
                'Alignment', 'right');
            axis1Group = matlab.system.display.SectionGroup(...
                'Title', 'Axis 1', 'Sections', [axis1Group1, axis1Group2]);
            groups = [configGroup, axis0Group, axis1Group];
        end
        
        function flag = isInactivePropertyImpl(obj, propertyName)
            ret = Simulink.Mask.get(gcb);
            if ~isempty(ret)
                names = {ret.Parameters.Name};
                flag = false;
                disable = false;
                if strcmp(propertyName, 'portName')
                    flag = ~obj.isPort;
                elseif strcmp(propertyName, 'serial')
                    flag = obj.isPort;
                elseif strcmp(propertyName, 'ctrlMode0')
                    disable = ~obj.isAxis0;
                elseif strcmp(propertyName, 'isInCpr0')
                    disable = ~obj.isAxis0 | ~strcmp(obj.ctrlMode0, 'Position');
                elseif strcmp(propertyName, 'isVelRamp0')
                    disable = ~obj.isAxis0 | ~strcmp(obj.ctrlMode0, 'Velocity');
                elseif strcmp(propertyName, 'watchdog0')
                    disable = ~obj.isAxis0;
                elseif strcmp(propertyName, 'isExternal0')
                    disable = ~obj.isAxis0;
                elseif strcmp(propertyName, 'posGain0')
                    disable = ~obj.isAxis0 | ~strcmp(obj.ctrlMode0, 'Position') | obj.isExternal0;
                elseif strcmp(propertyName, 'velGain0')
                    disable = ~obj.isAxis0 | strcmp(obj.ctrlMode0, 'Current') | obj.isExternal0;
                elseif strcmp(propertyName, 'velIGain0')
                    disable = ~obj.isAxis0 | strcmp(obj.ctrlMode0, 'Current') | obj.isExternal0;
                elseif strcmp(propertyName, 'velLimit0')
                    disable = ~obj.isAxis0 | strcmp(obj.ctrlMode0, 'Current') | obj.isExternal0;
                elseif strcmp(propertyName, 'velLimitTol0')
                    disable = ~obj.isAxis0 | strcmp(obj.ctrlMode0, 'Current') | obj.isExternal0;
                elseif strcmp(propertyName, 'velRampRate0')
                    disable = ~obj.isAxis0 | strcmp(obj.ctrlMode0, 'Current') | obj.isExternal0;
                elseif strcmp(propertyName, 'ctrlMode1')
                    disable = ~obj.isAxis1;
                elseif strcmp(propertyName, 'isInCpr1')
                    disable = ~obj.isAxis1 | ~strcmp(obj.ctrlMode1, 'Position');
                elseif strcmp(propertyName, 'isVelRamp1')
                    disable = ~obj.isAxis1 | ~strcmp(obj.ctrlMode1, 'Velocity');
                elseif strcmp(propertyName, 'watchdog1')
                    disable = ~obj.isAxis1;
                elseif strcmp(propertyName, 'isExternal1')
                    disable = ~obj.isAxis1;
                elseif strcmp(propertyName, 'posGain1')
                    disable = ~obj.isAxis1 | ~strcmp(obj.ctrlMode1, 'Position') | obj.isExternal1;
                elseif strcmp(propertyName, 'velGain1')
                    disable = ~obj.isAxis1 | strcmp(obj.ctrlMode1, 'Current') | obj.isExternal1;
                elseif strcmp(propertyName, 'velIGain1')
                    disable = ~obj.isAxis1 | strcmp(obj.ctrlMode1, 'Current') | obj.isExternal1;
                elseif strcmp(propertyName, 'velLimit1')
                    disable = ~obj.isAxis1 | strcmp(obj.ctrlMode1, 'Current') | obj.isExternal1;
                elseif strcmp(propertyName, 'velLimitTol1')
                    disable = ~obj.isAxis1 | strcmp(obj.ctrlMode1, 'Current') | obj.isExternal1;
                elseif strcmp(propertyName, 'velRampRate1')
                    disable = ~obj.isAxis1 | strcmp(obj.ctrlMode1, 'Current') | obj.isExternal1;
                else
                    flag = false;
                end
                
                if disable
                    ret.Parameters(find(strcmp(names, propertyName))).Enabled = 'off';
                else
                    ret.Parameters(find(strcmp(names, propertyName))).Enabled = 'on';
                end
            else    
                flag = false;
            end
        end
    end
    
    methods
        function checkOdriveSerialNumber(obj)
            mypi = raspi();
            ret = system(mypi,'(sudo lsusb -d 1209:0d32 -v; sudo lsusb -d 0483:df11 -v) 2>/dev/null | grep iSerial; echo');
            ret = erase(ret,"  iSerial                 3 ");
            ret = splitlines(ret);
            opts = struct('WindowStyle', 'modal', 'Interpreter', 'tex');
            if isempty(ret{1})
                msgbox('\bf \fontsize{10} \color{red} Not detected', 'Serial number', 'help', opts);
            else
                msgbox(['\bf \fontsize{10} \color{blue}', ret{1:end-2}], 'Serial number', 'help', opts);
            end
        end
        
        function getOdriveAxis0Configs(obj)
            val = loadOdriveConfigs(obj);
            if isempty(val)
                return;
            end
            set_param(gcb, 'posGain0', num2str(val.axis0.controller.config.pos_gain));
            set_param(gcb, 'velGain0', num2str(val.axis0.controller.config.vel_gain));
            set_param(gcb, 'velIGain0', num2str(val.axis0.controller.config.vel_integrator_gain));
            set_param(gcb, 'velLimit0', num2str(val.axis0.controller.config.vel_limit));
            set_param(gcb, 'velLimitTol0', num2str(val.axis0.controller.config.vel_limit_tolerance));
            set_param(gcb, 'velRampRate0', num2str(val.axis0.controller.config.vel_ramp_rate));
        end
        
        function getOdriveAxis1Configs(obj)
            val = loadOdriveConfigs(obj);
            if isempty(val)
                return;
            end
            set_param(gcb, 'posGain1', num2str(val.axis1.controller.config.pos_gain));
            set_param(gcb, 'velGain1', num2str(val.axis1.controller.config.vel_gain));
            set_param(gcb, 'velIGain1', num2str(val.axis1.controller.config.vel_integrator_gain));
            set_param(gcb, 'velLimit1', num2str(val.axis1.controller.config.vel_limit));
            set_param(gcb, 'velLimitTol1', num2str(val.axis1.controller.config.vel_limit_tolerance));
            set_param(gcb, 'velRampRate1', num2str(val.axis1.controller.config.vel_ramp_rate));
        end
        
        function val = loadOdriveConfigs(obj)
            opts = struct('WindowStyle', 'modal', 'Interpreter', 'tex');
            if obj.isPort
                msgbox('\fontsize{10} It cannot be used when "Enable specify serial port".', 'Load ODrive configuration', 'error', opts);
                return;
            end
            f = waitbar(0, 'Loading ODrive''s configuration. Please wait...', 'Name', 'Progress');
            mypi = raspi();
            waitbar(.25, f);
            ret1 = splitlines(system(mypi,'ls -1 /tmp | grep odrive-config-; echo'));
            if ~isempty(ret1{1})
                for i = 1:length(ret1)
                    if ~isempty(ret1{i})
                        system(mypi, ['mv /tmp/', ret1{i}, ' /tmp/temp_', ret1{i}]);
                    end
                end
            end
            system(mypi, 'odrivetool backup-config');
            fileName = ['odrive-config-', obj.serial, '.json'];
            filePath = ['/tmp/', fileName];
            ret2 = system(mypi, ['test -e ', filePath, '; echo $?']);
            if strcmp(ret2(1),'1')
                if ~isempty(ret1{1})
                    for i = 1:length(ret1)
                        if ~isempty(ret1{i})
                            system(mypi, ['mv /tmp/temp_', ret1{i}, ' /tmp/', ret1{i}]);
                        end
                    end
                end
                msgbox(['\fontsize{10} Failed to load configuration : The specified ODrive''s serial number  "', obj.serial, '"  does not exist.'], 'Error', 'error', opts);
                close(f);
                val = [];
                return;
            end
            waitbar(.75, f);
            getFile(mypi, filePath);
            system(mypi, ['rm ', filePath]);
            if ~isempty(ret1{1})
                for i = 1:length(ret1)
                    if ~isempty(ret1{i})
                        system(mypi, ['mv /tmp/temp_', ret1{i}, ' /tmp/', ret1{i}]);
                    end
                end
            end
            waitbar(1, f);
            val = jsondecode(fileread(fileName));
            close(f);
        end
    end
    
    methods (Static)
        function name = getDescriptiveName()
            name = 'ODriveRasPi';
        end
        
        function b = isSupportedContext(context)
            b = context.isCodeGenTarget('rtw');
        end
        
        function updateBuildInfo(buildInfo, context)
            if context.isCodeGenTarget('rtw')
                % Update buildInfo
                srcDir = fullfile(fileparts(mfilename('fullpath')),'src'); %#ok<NASGU>
                includeDir = fullfile(fileparts(mfilename('fullpath')),'include');
                addIncludePaths(buildInfo,includeDir);
                % Use the following API's to add include files, sources and
                % linker flags
                addSourceFiles(buildInfo,'odrive_raspi.c', srcDir);
            end
        end
    end
end
