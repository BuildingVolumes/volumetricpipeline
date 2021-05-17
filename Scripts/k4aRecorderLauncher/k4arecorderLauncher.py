import PySimpleGUI as sg
import os
import json

rgbmode = "";
depthmode = "";

SyncMode = "";
SyncIndex = "";

OutputFolder = "";
FilePrefix = "";
Filename = FilePrefix + SyncIndex + ".mkv"
CMD = "";
fps = "";
config = {"rgbmode":rgbmode, "depthmode":depthmode, "SyncMode":SyncMode, "SyncIndex":SyncIndex, "OutputFolder":OutputFolder, "FilePrefix":FilePrefix, "CMD":CMD, "fps":fps}


def write_config_file():
    global config
    #config = {"rgbmode":rgbmode, "depthmode":depthmode, "SyncMode":SyncMode, "SyncIndex":SyncIndex, "OutputFolder":OutputFolder, "FilePrefix":FilePrefix, "CMD":CMD, "fps":fps}
    with open('config1.json','w') as f:
        json.dump(config,f);

def read_config_file():
    global config
    with open('config1.json','r') as f:
        config = json.load(f);
    

def print_all():
    global config
    print('rgbmode:'+config['rgbmode'])
    print('depthmode:'+config['depthmode'])
    print('SyncMode:'+config['SyncMode'])
    print('SyncIndex:'+config['SyncIndex'])
    print('OutputFolder:'+config['OutputFolder'])
    print('FilePrefix:'+config['FilePrefix'])
    print('CMD:'+config['CMD'])
    print('fps:'+config['fps'])


def set_all(valuesT):
    global config;
    print(config)
    
    config['rgbmode'] = valuesT[1];
    config['depthmode'] = valuesT[2];
    config['SyncMode'] = valuesT[3];
    config['SyncIndex'] = valuesT[4];
    config['OutputFolder'] = valuesT[5];
    config['FilePrefix'] = valuesT[6];
    config['CMD'] = valuesT[7];
    config['fps'] = valuesT[8];
    print_all()



# always read the config first
read_config_file();
#print_all()






# ------ Menu Definition ------ #
menu_def = [['&File', ['&Open', '&Save', 'E&xit', 'Properties']],
            ['&Edit', ['Paste', ['Special', 'Normal', ], 'Undo'], ],
            ['&Help', '&About...'], ]
layout = [
    [sg.Menu(menu_def, tearoff=True)],
    [sg.Text('Azure Kinect Recorder (k4arecorder) Settings')],
    [sg.Frame(layout=[
        [sg.Text('RGB   Cam Mode:',  size=(20, 1)), sg.InputOptionMenu(('1080p', '3072p', '2160p', '1536p','1440p','720p','720p_NV12','720p_YUY2','OFF'), size = (15,2), default_value=config['rgbmode'])],
        [sg.Text('DEPTH Cam Mode:',  size=(20, 1)), sg.InputOptionMenu(('NFOV_2x2BINNED', 'NFOV_UNBINNED', 'WFOV_2x2BINNED', 'WFOV_UNBINNED','PASSIVE_IR','720p','720p_NV12','720p_YUY2','OFF'), size = (15,2), default_value=config['depthmode'] )],
    ], title='Camera Settings',title_color='white', relief=sg.RELIEF_SUNKEN, tooltip='Use these to set flags')],
    [sg.Frame(layout=[
        [sg.Text('SyncMode',  size=(20, 1)), sg.InputOptionMenu(('Standalone','Master','Subordinate'), size = (10,2), default_value=config['SyncMode'])],
        [sg.Text('Daisy Chain Index:',  size=(20, 1)), sg.InputOptionMenu(('0', '1', '2', '3','4','5','6','7','8','9'), size = (10,2), default_value=config['SyncIndex'])],
    ], title='Synchronization Settings',title_color='white', relief=sg.RELIEF_SUNKEN, tooltip='Use these to set flags')],
    [sg.Frame(layout=[
        [sg.Text('Output Folder', size=(20, 1), auto_size_text=False, justification='left'), sg.InputText(config['OutputFolder']), sg.FolderBrowse()],
        [sg.Text('MKV File Prefix', size=(20, 1)), sg.InputText(config['FilePrefix'])],
        [sg.Text('K4Arecorder Executable', size=(20, 1), auto_size_text=False, justification='left'), sg.InputText(config['CMD']), sg.FolderBrowse()],
        [sg.Text('Framerate',  size=(20, 1)), sg.InputOptionMenu(('30', '15', '5'), size = (15,2), default_value=config['fps'])],
    ], title='Misc Settings',title_color='white', relief=sg.RELIEF_SUNKEN, tooltip='Use these to set flags')],
    [sg.Button(button_text='Save Config')],
    [sg.Submit(tooltip='Click to submit this form'), sg.Cancel()]]



## the main loop 
window = sg.Window('Azure Kinect Recorder Launcher', layout, default_element_size=(40, 1), grab_anywhere=False)
while True:
    event, values = window.Read()
    print(event, values)
    set_all(values);
    print_all();

    if event == 'Save Config':
        write_config_file();
    if event == 'Cancel':
        break;
    if event == 'Submit':
        #run the command 
        delay = str(160*int(config['SyncIndex']))
        Filename = config['FilePrefix'] + config['SyncIndex'] + ".mkv"
        Command = "\""+config['CMD'] +"\""+ " " + "-c " + config['rgbmode'] + " -d " + config['depthmode'] + " --depth-delay 0" + " -r "+config['fps'] + " --external-sync " + config['SyncMode'] + " --sync-delay "+delay + " " + Filename;
        print("COMMAND="+Command);
        os.system(Command)
        break;

window.close()

