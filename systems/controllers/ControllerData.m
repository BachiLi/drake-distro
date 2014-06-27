classdef (Abstract) ControllerData < handle
  % ControllerData is an abstract class designed to be the parent of
  % classes that contain data shared between controller modules. It is a
  % handle class because some of these properties change during execution
  % and it is desirable for all modules' references to be updated.
  
  % optional: for properties that change infrequently or never
  properties (SetAccess=private,GetAccess=public)

  end
  
  % optional: for properties that can be modified 'on the fly'
  properties (SetAccess=public,GetAccess=public)
  
  end
  
  methods (Abstract)
    % asserts existence and type of properties contained in 'data'
    verifyControllerData(obj,data)
  end
end