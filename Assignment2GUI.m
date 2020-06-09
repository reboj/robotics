function varargout = Assignment2GUI(varargin)
% ASSIGNMENT2GUI MATLAB code for Assignment2GUI.fig
%      ASSIGNMENT2GUI, by itself, creates a new ASSIGNMENT2GUI or raises the existing
%      singleton*.
%
%      H = ASSIGNMENT2GUI returns the handle to a new ASSIGNMENT2GUI or the handle to
%      the existing singleton*.
%
%      ASSIGNMENT2GUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in ASSIGNMENT2GUI.M with the given input arguments.
%
%      ASSIGNMENT2GUI('Property','Value',...) creates a new ASSIGNMENT2GUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before Assignment2GUI_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to Assignment2GUI_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help Assignment2GUI

% Last Modified by GUIDE v2.5 08-Jun-2020 20:14:21

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @Assignment2GUI_OpeningFcn, ...
                   'gui_OutputFcn',  @Assignment2GUI_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before Assignment2GUI is made visible.
function Assignment2GUI_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to Assignment2GUI (see VARARGIN)

% Choose default command line output for Assignment2GUI
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);
% Here, add the links to the run code

% UIWAIT makes Assignment2GUI wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = Assignment2GUI_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in Trajectory.
function Trajectory_Callback(hObject, eventdata, handles)
%Pressing this GUI will result in running the trajectory of the file
% hObject    handle to Trajectory (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
Trajectory;
%run('D:\roboticstoolbox (do not delete)\robot-9.10Small_Modified_20180320_WithVisionTB (1)\robot-9.10Small_Modified\rvctools\assignment2.m')

% --- Executes on button press in Collision.
function Collision_Callback(hObject, eventdata, handles)
% hObject    handle to Collision (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in Joystick.
function Joystick_Callback(hObject, eventdata, handles)
% hObject    handle to Joystick (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in STOP.
function STOP_Callback(hObject, eventdata, handles)
% hObject    handle to STOP (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%set(hObject,'Value',1)

% --- Executes during object creation, after setting all properties.
function Trajectory_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Trajectory (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
