pkg load control signal
graphics_toolkit qt

close all
clear all
clc

global model;
global h;
global response_opt = 1;
global analisys_opt = 1;

model.laminar = true;
model.closed_loop = true;
model.ctrl_var = 'Level';   # or Flow Velocity
model.ctrl_type = 'Custom';

            # DESCRIPTION                       : UNITS
model.Q = 1;      # Fluid velocity                    : m**3 / seg
model.Kl = 0.071; # Pipe coefficent (laminar)         : m**2 / seg
model.Kt = 0.033; # Pipe coefficent (turbulent)       : m**2.5 / seg
model.He = 1;     # Height in steady state            : m
model.V = 1;      # Volumn of the liquid in the tank  : m**3

# Small changes from the steady state point.
model.dQ = 0.05;    # Fluid velocity                    : m**3 / seg
model.dH = 0.01;    # Height in steady state            : m
model.dV = 0.007;   # Volumn of the liquid in the tank  : m**3

model.Rl = model.dH / model.dQ;     # Pipe resistance (laminar)   : seg / m**2
model.Rt = 2 *model.dH / model.dQ;  # Pipe resistance (turbulent) : seg / m**2
model.C = model.dV / model.dH;      # Tank Capacitance            : m**2

# Controller Parameters
model.numc = [5];
model.denc = [1 1 0];
model.Kp = 1;
model.Ki = 1;
model.Kd = 0;

# Feedback Parameters (sensor, amplifier, noise filter)
model.numh = [1];
model.denh = [1];

# Parameters for a typical TANK LEVEL / FLUID VELOCITY laminar plant.
model.nump = [model.Rl];
model.denp = [(model.Rl * model.C) 1];

function build_model
  
  global model;
  global h;
  
  # Transfer function for a typical TANK LEVEL / FLUID VELOCITY laminar plant.
  if length (model.nump) <= length (model.denp)
    model.Gp = tf (model.nump, model.denp);
  endif

  # Controller transfer function
  if ((length (model.numc) <= length (model.denc)) && get (h.Custom_opt, 'value'))
    model.Gc = tf (model.numc, model.denc);
  elseif (get (h.Parallel_opt, 'value'))
    P = 0;
    I = 0;
    D = 0;
    if (get (h.P_enabled, 'value') == true)
      P = model.Kp;
    endif
    if (get (h.I_enabled, 'value') == true)
      I = model.Ki;
    endif
    if (get (h.D_enabled, 'value') == true)
      D = model.Kd;
    endif
    
    model.Gc = pid (P, I, D);
  endif

  # Feedback transfer function (sensor, amplifier, noise filter)
  if (length (model.numh) <= length (model.denh))
    model.H = tf (model.numh, model.denh);
  endif
  
  # Transfer function of direct path
  model.G = series(model.Gc, model.Gp)
  
  if model.closed_loop == true
    # Transfer function of the whole system
    model.sys = feedback (model.G, model.H);
  else
    model.sys = model.G;
  endif
  
  plot_response ();
  plot_analisys();
  
endfunction


function update_params
  
  global model;
  global h;
  
  model.Rl = model.dH / model.dQ;       # Pipe resistance (laminar)   : seg / m**2
  model.Rt = 2 *model.dH / model.dQ;     # Pipe resistance (turbulent) : seg / m**2
  model.C = model.dV / model.dH;        # Tank Capacitance            : m**2

  # Parameters for a typical TANK LEVEL / FLUID VELOCITY laminar plant.
  if (model.laminar)
    model.nump = [model.Rl];
    model.denp = [(model.Rl * model.C) 1];
  else
    model.nump = [model.Rt];
    model.denp = [(model.Rt * model.C) 1];
  endif
  
  set (h.plant_num, 'string', num2str (model.nump));
  set (h.plant_den, 'string', num2str (model.denp));
  
  build_model ();
  
endfunction

function plot_response
  
  global model;
  global response_opt;
  
  subplot ("position", [0.68 0.55 0.3 0.35]);
      
  switch response_opt
    case 1
      [y, t, _] = impulse (model.sys);
      plot (t, model.He * y, '-r', t, zeros (length (t), 1), '--b')
      grid
      hold off
    case 2
      [y, t, _] = step (model.sys);
      plot (t, model.He * y, '-r', t, model.He * ones (length (t), 1), '--b')
      grid
    case 3
      [y, t, _] = ramp (model.sys);
      plot (t, model.He * y, '-r', t, model.He * t, '--b')
      grid
  endswitch
  
endfunction

function plot_analisys
  
  global model;
  global analisys_opt;
  
  subplot ("position", [0.68 0.07 0.3 0.33]); #[0.3 0.43 0.3 0.53]
      
  switch (analisys_opt)
    case 1
      rlocus(model.sys)
      sgrid
      
    case 2
      subplot ("position", [0.68 0.27 0.3 0.164]);
      [mag, pha, w] = bode(model.sys);
      semilogx (w, 20*log10(mag));
      grid
      
      subplot ("position", [0.68 0.06 0.3 0.164]);
      [mag, pha, w] = bode(model.sys);
      semilogx (w, pha);
      grid
      
    case 3
      nyquist(model.sys)
      
    case 4
      nichols(model.sys)
      
  endswitch
      
endfunction

function update_plots (obj, init = false)
  
  global model;
  global response_opt;
  global analisys_opt;
  
  ## gcbo holds the handle of the control
  h = guidata (obj);
  
  switch (gcbo)
    
    case {h.response_list} 
      response_opt = get (h.response_list, 'value');
      plot_response ();
            
    case {h.analisys_list} 
      analisys_opt = get (h.analisys_list, 'value');
      plot_analisys ();
    
    case {h.Custom_opt}
      build_model ();
    case {h.Parallel_opt}
      build_model ();
    case {h.Series_opt}
      build_model ();
      
    case {h.P_enabled}
      build_model ();
    case {h.I_enabled}
      build_model ();
    case {h.D_enabled}
      build_model ();
      
    case {h.ctrl_num}
      model.numc = str2num (get (h.ctrl_num, 'string'));
      build_model ();
      
    case {h.ctrl_den}
      model.denc = str2num (get (h.ctrl_den, 'string'));
      build_model ();
      
    case {h.P_value}
      model.Kp = str2num (get (h.P_value, 'string'));
      build_model ();
    
    case {h.I_value}
      model.Ki = str2num (get (h.I_value, 'string'));
      build_model ();
    
    case {h.D_value}
      model.Kd = str2num (get (h.D_value, 'string'));
      build_model ();
    
    case {h.retro_num}
      model.numh = str2num (get (h.retro_num, 'string'));
      build_model ();
      
    case {h.retro_den}
      model.denh = str2num (get (h.retro_den, 'string'));
      build_model ();
    
    case {h.plant_num}
      model.nump = str2num (get (h.plant_num, 'string'));
      build_model ();
      
    case {h.plant_den}
      model.denp = str2num (get (h.plant_den, 'string'));
      build_model ();
      
    case {h.fv_nominal}
      model.Q = str2num (get (h.fv_nominal, 'string'));
      update_params();
    
    case {h.ll_nominal}
      model.He = str2num (get (h.ll_nominal, 'string'));
      update_params();
    
    case {h.fv_deviation}
      model.dQ = str2num (get (h.fv_deviation, 'string'));;    
      update_params ();
    
    case {h.ll_deviation}
      model.dH = str2num (get (h.ll_deviation, 'string'));;    
      update_params ();
    
    case {h.ft_laminar}
      model.laminar = true;    
      update_params ();
    
    case {h.ft_turbulent}
      model.laminar = false;    
      update_params ();
      
  endswitch
  
endfunction


## Creatge Dialog
h.fig = figure ('position', [100 45 1200 700], 'menubar', 'none');

#{
subplot ("position", [0.0 0.10 0.5 0.5]); 
h.I = imread ('/home/Huitzyl/Imágenes/CL.png');
h.img = imshow (h.I);
axis off
#}
#{
subplot ("position", [0.0 0.03 0.7 0.9]);
line ([0.5 0.58], [0.67 0.67], 'linestyle', '-', 'color', 'k', 'marker', '>', 'linewidth', 4);
line ([0.1 0.39], [0.37 0.37], 'linestyle', '-', 'color', 'k', 'marker', '<', 'linewidth', 4);
line ([0.7 0.95], [0.37 0.37], 'linestyle', '-', 'color', 'k', 'marker', '<', 'linewidth', 4);
ylim ([0 1]);
xlim ([0 1]);
axis off;
axis square;
#}

# Response type selection
h.response_list = uicontrol ("style", "popupmenu",
                                "units", "normalized",
                                "callback", @update_plots,
                                "position", [0.83 0.92 0.1 0.05],
                                "string", { 'Impulse', 
                                            'Step',
                                            'Ramp'});

h.response_label = uicontrol ("style", "text",
                           "units", "normalized",
                           "string", "Response to:",
                           "horizontalalignment", "left",
                           "position", [0.73 0.92 0.08 0.05]);
                           
set (h.response_list, 'value', response_opt);


# Analisys type selection
h.analisys_list = uicontrol ("style", "popupmenu",
                                "units", "normalized",
                                "callback", @update_plots,
                                "position", [0.83 0.445 0.1 0.05],
                                "string", { 'Root Locus', 
                                            'Bode',
                                            'Nyquist',
                                            'Nichols'});

h.analisys_label = uicontrol ("style", "text",
                           "units", "normalized",
                           "string", "Analisys Method:",
                           "horizontalalignment", "left",
                           "position", [0.73 0.445 0.095 0.05]);

set (h.analisys_list, 'value', analisys_opt);










# CONTROLLER
h.ctrl_panel = uibuttongroup (h.fig, 'title', 'Controller configuration',
                    'units', 'normalized',
                    'position', [0.34 0.43 0.3 0.53]);

# MORE
h.Custom_opt = uicontrol (h.ctrl_panel, 'style', 'radiobutton',
                    'string', 'Custom Controller',
                    'units', 'normalized',
                    'position', [0.3 0.85 0.4 0.125],
                    'callback', @update_plots); 

set (h.Custom_opt, 'value', true);
                    
h.ctrl_num_label = uicontrol (h.ctrl_panel, 'style', 'text',
                    'units', 'normalized',
                    'string', 'num:',
                    'horizontalalignment', 'left',
                    'position', [0.06 0.72 0.65 0.125]);
                    
h.ctrl_num = uicontrol (h.ctrl_panel, 'style', 'edit',
                    'units', 'normalized',
                    'position', [0.25 0.72 0.68 0.125],
                    'callback', @update_plots);
                    
set (h.ctrl_num, 'string', num2str (model.numc));
                  
h.ctrl_den_label = uicontrol (h.ctrl_panel, 'style', 'text',
                    'units', 'normalized',
                    'string', 'den:',
                    'horizontalalignment', 'left',
                    'position', [0.06 0.58 0.65 0.125]);
                    
h.ctrl_den = uicontrol (h.ctrl_panel, 'style', 'edit',
                    'units', 'normalized',
                    'position', [0.25 0.58 0.68 0.125],
                    'callback', @update_plots);
                    
set (h.ctrl_den, 'string', num2str (model.denc));

# PID                  

h.Parallel_opt = uicontrol (h.ctrl_panel, 'style', 'radiobutton',
                    'string', 'PID Parallel',
                    'units', 'normalized',
                    'position', [0.2 0.43 0.3 0.125],
                    'callback', @update_plots);  

h.Series_opt = uicontrol (h.ctrl_panel, 'style', 'radiobutton',
                    'enable', 'off',
                    'string', 'PID Series',
                    'units', 'normalized',
                    'position', [0.5 0.43 0.3 0.125],
                    'callback', @update_plots);  
                    
h.P_enabled = uicontrol (h.ctrl_panel, 'style', 'checkbox',
                    'string', 'Proportional',
                    'units', 'normalized',
                    'position', [0.05 0.3 0.3 0.125],
                    'callback', @update_plots);
set (h.P_enabled, 'selected', true);
                    
h.P_value = uicontrol (h.ctrl_panel, 'style', 'edit',
                    'units', 'normalized',
                    'position', [0.45 0.3 0.5 0.125],
                    'callback', @update_plots);                    
set (h.P_value, 'string', num2str (model.Kp));                    

h.I_enabled = uicontrol (h.ctrl_panel, 'style', 'checkbox',
                    'string', 'Integral',
                    'units', 'normalized',
                    'position', [0.05 0.16 0.3 0.125],
                    'callback', @update_plots);
set (h.I_enabled, 'value', false);
                    
h.I_value = uicontrol (h.ctrl_panel, 'style', 'edit',
                    'units', 'normalized',
                    'position', [0.45 0.16 0.5 0.125],
                    'callback', @update_plots); 
set (h.I_value, 'string', num2str (model.Ki));
                    
h.D_enabled = uicontrol (h.ctrl_panel, 'style', 'checkbox',
                    'string', 'Derivative',
                    'units', 'normalized',
                    'position', [0.05 0.02 0.3 0.125],
                    'callback', @update_plots);
set (h.P_enabled, 'value', false);
                    
h.D_value = uicontrol (h.ctrl_panel, 'style', 'edit',
                    'units', 'normalized',
                    'position', [0.45 0.02 0.5 0.125],
                    'callback', @update_plots);                     
set (h.D_value, 'string', num2str (model.Kd));
                    
                    
# Retro UI = H(s)               
h.retro_panel = uipanel (h.fig, 'title', 'Feedback T.F. = H(s)',
                    'units', 'normalized',
                    'position', [0.34 0.2 0.3 0.18]); #[0.3 0.43 0.3 0.53]

h.retro_num_label = uicontrol (h.retro_panel, 'style', 'text',
                    'units', 'normalized',
                    'string', 'num:',
                    'horizontalalignment', 'left',
                    'position', [0.06 0.5 0.65 0.41]);
                    
h.retro_num = uicontrol (h.retro_panel, 'style', 'edit',
                    'units', 'normalized',
                    'position', [0.25 0.5 0.68 0.4],
                    'callback', @update_plots);
                    
set (h.retro_num, 'string', num2str (model.numh));
                  
h.retro_den_label = uicontrol (h.retro_panel, 'style', 'text',
                    'units', 'normalized',
                    'string', 'den:',
                    'horizontalalignment', 'left',
                    'position', [0.06 0.05 0.65 0.41]);
                    
h.retro_den = uicontrol (h.retro_panel, 'style', 'edit',
                    'units', 'normalized',
                    'position', [0.25 0.05 0.68 0.4],
                    'callback', @update_plots);
                    
set (h.retro_den, 'string', num2str (model.denh));

# Tank UI (Plant)              
h.plant_panel = uipanel (h.fig, 'title', 'Plant T.F. = G(s)',
                    'units', 'normalized',
                    'position', [0.025 0.2 0.3 0.18]);%[0.025 0.43 0.25 0.53]

h.plant_num_label = uicontrol (h.plant_panel, 'style', 'text',
                    'units', 'normalized',
                    'string', 'num:',
                    'horizontalalignment', 'left',
                    'position', [0.06 0.5 0.65 0.41]);
                    
h.plant_num = uicontrol (h.plant_panel, 'style', 'edit',
                    'units', 'normalized',
                    'position', [0.25 0.5 0.68 0.4],
                    'callback', @update_plots);
                          
set (h.plant_num, 'string', num2str (model.nump));

h.plant_den_label = uicontrol (h.plant_panel, 'style', 'text',
                    'units', 'normalized',
                    'string', 'den:',
                    'horizontalalignment', 'left',
                    'position', [0.06 0.05 0.65 0.41]);
                    
h.plant_den = uicontrol (h.plant_panel, 'style', 'edit',
                    'units', 'normalized',
                    'position', [0.25 0.05 0.68 0.4],
                    'callback', @update_plots);
                          
set (h.plant_den, 'string', num2str (model.denp));







# Tank Physical Params UI (Plant)              
h.process_panel = uipanel (h.fig, 'title', 'Process Parameters',
                    'units', 'normalized',
                    'position', [0.025 0.43 0.3 0.53]);

# Fluid Velocity (Container)                   
h.fluid_vel_panel = uipanel (h.process_panel, 'title', 'Fliud Velocity',
                    'units', 'normalized',
                    'position', [0.05 0.65 0.88 0.3]);

# Fluid Velocity (Nominal)    
h.fv_nominal_label = uicontrol (h.fluid_vel_panel, 'style', 'text',
                    'units', 'normalized', # m**3 / seg
                    'string', 'Nominal:',
                    'horizontalalignment', 'left',
                    'position', [0.06 0.6 0.65 0.2]);
                    
h.fv_nominal = uicontrol (h.fluid_vel_panel, 'style', 'edit',
                    'units', 'normalized',
                    'position', [0.36 0.55 0.25 0.3],
                    'callback', @update_plots);
                    
h.fv_units_label = uicontrol (h.fluid_vel_panel, 'style', 'text',
                    'units', 'normalized', # m**3 / seg
                    'string', 'm**3 / seg',
                    'horizontalalignment', 'left',
                    'position', [0.66 0.6 0.65 0.2]);
                          
set (h.fv_nominal, 'string', num2str (model.Q));

# Fluid Velocity (Deviation)
h.fv_deviation_label = uicontrol (h.fluid_vel_panel, 'style', 'text',
                    'units', 'normalized', # m**3 / seg
                    'string', 'Deviation:',
                    'horizontalalignment', 'left',
                    'position', [0.06 0.2 0.65 0.2]);
                    
h.fv_deviation = uicontrol (h.fluid_vel_panel, 'style', 'edit',
                    'units', 'normalized',
                    'position', [0.36 0.15 0.25 0.3],
                    'callback', @update_plots);
                    
h.fvd_units_label = uicontrol (h.fluid_vel_panel, 'style', 'text',
                    'units', 'normalized', # m**3 / seg
                    'string', 'm**3 / seg',
                    'horizontalalignment', 'left',
                    'position', [0.66 0.2 0.65 0.2]);
                          
set (h.fv_deviation, 'string', num2str (model.dQ));









# Liquid Level (Container)                   
h.liq_level_panel = uipanel (h.process_panel, 'title', 'Liquid Level',
                    'units', 'normalized',
                    'position', [0.05 0.35 0.88 0.3]);

# Liquid Level (Nominal)    
h.ll_nominal_label = uicontrol (h.liq_level_panel, 'style', 'text',
                    'units', 'normalized', # m**3 / seg
                    'string', 'Nominal:',
                    'horizontalalignment', 'left',
                    'position', [0.06 0.6 0.65 0.2]);
                    
h.ll_nominal = uicontrol (h.liq_level_panel, 'style', 'edit',
                    'units', 'normalized',
                    'position', [0.36 0.55 0.25 0.3],
                    'callback', @update_plots);
                    
h.ll_units_label = uicontrol (h.liq_level_panel, 'style', 'text',
                    'units', 'normalized', # m**3 / seg
                    'string', 'm',
                    'horizontalalignment', 'left',
                    'position', [0.66 0.6 0.65 0.2]);
                          
set (h.ll_nominal, 'string', num2str (model.He));

# Liquid Level (Deviation)
h.ll_deviation_label = uicontrol (h.liq_level_panel, 'style', 'text',
                    'units', 'normalized', # m**3 / seg
                    'string', 'Deviation:',
                    'horizontalalignment', 'left',
                    'position', [0.06 0.2 0.65 0.2]);
                    
h.ll_deviation = uicontrol (h.liq_level_panel, 'style', 'edit',
                    'units', 'normalized',
                    'position', [0.36 0.15 0.25 0.3],
                    'callback', @update_plots);
                    
h.lld_units_label = uicontrol (h.liq_level_panel, 'style', 'text',
                    'units', 'normalized', # m**3 / seg
                    'string', 'm',
                    'horizontalalignment', 'left',
                    'position', [0.66 0.2 0.65 0.2]);
                          
set (h.ll_deviation, 'string', num2str (model.dH));






# Flow Reynolds (Container)                   
h.flow_type_panel = uibuttongroup (h.process_panel, 'title', 'Flow Type',
                    'units', 'normalized',
                    'position', [0.05 0.05 0.88 0.3]);

h.ft_laminar = uicontrol (h.flow_type_panel, 'style', 'radiobutton',
                    'units', 'normalized', # m**3 / seg
                    'string', 'Laminar     Kl =',
                    'horizontalalignment', 'left',
                    'position', [0.03 0.6 0.65 0.2],
                    'callback', @update_plots);

set (h.ft_laminar, 'value', model.laminar);                    

h.Kl = uicontrol (h.flow_type_panel, 'style', 'edit',
                    'units', 'normalized',
                    'position', [0.52 0.55 0.25 0.3],
                    'callback', @update_plots);
                    
set (h.Kl, 'string', num2str (model.Kl));

h.ft_turbulent = uicontrol (h.flow_type_panel, 'style', 'radiobutton',
                    'units', 'normalized', # m**3 / seg
                    'string', 'Turbulent   Kt =',
                    'horizontalalignment', 'left',
                    'position', [0.03 0.2 0.65 0.2],
                    'callback', @update_plots);

h.Kt = uicontrol (h.flow_type_panel, 'style', 'edit',
                    'units', 'normalized',
                    'position', [0.52 0.15 0.25 0.3],
                    'callback', @update_plots);
                    
set (h.Kt, 'string', num2str (model.Kt));

#{
h.ft_turbulent = uicontrol (h.flow_type_panel, 'style', 'text',
                    'units', 'normalized',
                    'string', 'm**2.5 / seg',
                    'horizontalalignment', 'left',
                    'position', [0.63 0.2 0.65 0.2]);
#}                    
                    
                    

                    
build_model ();

set (gcf, "color", get(0, "defaultuicontrolbackgroundcolor"));
guidata (gcf, h);
#update_plots (gcf, true);


