#!/usr/bin/python

import os
import sys
import serial
import glob
from Tkinter import *
from ttk import *
from tkFileDialog import *
import Image
from functools import partial

FreertosArmv7mList = [['ek-lm4f120xl (legacy)', None, None, None],
                      ['ek-tm4c123gxl', None, None, None],
                      ['ek-tm4c1294xl', None, None, None],
                      ['Generic Tiva TM4C123', None, None, None],
                      ['Generic Tiva TM4C129', None, None, None],
                      ['Custom', None, None, None]]

# display help information in form of text
class ToolTip(object):

    def __init__(self, widget):
        self.widget = widget
        self.tipwindow = None
        self.id = None
        self.x = self.y = 0

    def showtip(self, text, y_offset):
        "Display text in tooltip window"
        self.text = text
        if self.tipwindow or not self.text:
            return
        x, y, cx, cy = self.widget.bbox("insert")
        x = x + self.widget.winfo_rootx() + 27
        y = y + cy + self.widget.winfo_rooty() + y_offset
        self.tipwindow = tw = Toplevel(self.widget)
        tw.wm_overrideredirect(1)
        tw.wm_geometry("+%d+%d" % (x, y))
        try:
            # For Mac OS
            tw.tk.call("::tk::unsupported::MacWindowStyle",
                       "style", tw._w,
                       "help", "noActivates")
        except TclError:
            pass
        label = Label(tw, text=self.text, justify=LEFT,
                      background="#ffffe0", relief=SOLID, borderwidth=1,
                      font=("tahoma", "8", "normal"))
        label.pack(ipadx=1)

    def hidetip(self):
        tw = self.tipwindow
        self.tipwindow = None
        if tw:
            tw.destroy()

def createToolTip(widget, text, y):
    toolTip = ToolTip(widget)
    def enter(event):
        toolTip.showtip(text, y)
    def leave(event):
        toolTip.hidetip()
    widget.bind('<Enter>', enter)
    widget.bind('<Leave>', leave)

# display help information in form of an image
class ToolImage(object):

    def __init__(self, widget):
        self.widget = widget
        self.imagewindow = None
        self.id = None
        self.x = self.y = 0

    def showimage(self, image):
        "Display image in toolimage window"
        self.image = image
        if self.imagewindow or not self.image:
            return
        x, y, cx, cy = self.widget.bbox("insert")
        x = x + self.widget.winfo_rootx() + 27
        y = y + cy + self.widget.winfo_rooty() + 27
        self.imagewindow = tw = Toplevel(self.widget)
        tw.wm_overrideredirect(1)
        tw.wm_geometry("+%d+%d" % (x, y))
        try:
            # For Mac OS
            tw.tk.call("::tk::unsupported::MacWindowStyle",
                       "style", tw._w,
                       "help", "noActivates")
        except TclError:
            pass
        photo = PhotoImage(file=self.image)
        label = Label(tw, image=photo)
        label.image = photo
        label.pack()

    def hideimage(self):
        tw = self.imagewindow
        self.imagewindow = None
        if tw:
            tw.destroy()

def createToolImage(widget, image):
    toolImage = ToolImage(widget)
    def enter(event):
        toolImage.showimage(image)
    def leave(event):
        toolImage.hideimage()
    widget.bind('<Enter>', enter)
    widget.bind('<Leave>', leave)

# The main application window
class App:
        

    #initialize main window
    def __init__(self, master):
        master.title("OpenMRN Application Builder")
        frame = Frame(master)
        frame.pack(fill=BOTH, expand=1)
        self.dir_opt = options = {}
        options['initialdir'] = 'C:\\'
        options['mustexist'] = False
        options['parent'] = root
        options['title'] = 'This is a title'
        ROW = 0

        #frame.columnconfigure(1, weight=1)
        #frame.columnconfigure(2, weight=3)
        #frame.columnconfigure(3, weight=1)
        #frame.rowconfigure(1, weight=10)
        #frame.rowconfigure(2, weight=1)

        # OpenMRN Path
        self.OpenmrnPath = None
        self.path_entry(frame, self.OpenmrnPath, 'OpenMRN Path',
                        os.path.abspath(os.getcwd() + '/../'),
                        'This is the file system path which points to\nthe '
                        'location of the OpenMRN project tree.',
                        0, 0)

        # Application Path
        self.ApplPath = None
        self.path_entry(frame, self.ApplPath, 'Application Path', None,
                        'This is the file system path which points to\nthe '
                        'location of the generated application.',
                        0, 2)

        ROW +=2

        # Application Sub-directories
        self.SubdirEntries = None
        self.path_entry(frame, self.SubdirEntries,
                        'Application Subdirectory List [space separated]',
                        None,
                        'Often it is usefull to add sub-directories\nto '
                        'better organize a large project.  The\nprovided '
                        'list of sub-directories will be\nadded to the '
                        'build rules for the\napplication.  Additional sub-'
                        'directories\ncan be added later by using the\n'
                        '"SubdirAdder" program.',
                        ROW, 0, False)

        # Template
        self.TemplateLabel = Label(frame, text="Template",
                                   font="Arial 12 bold")
        self.TemplateLabel.grid(row=ROW, column=2, sticky=S+W, padx=10)
        self.Template = Combobox(frame, textvariable=StringVar(), state="readonly")
        self.Template['values'] = ('empty', 'blink', 'train', 'traction proxy', 'hub')
        self.Template.current(0)
        self.Template.grid(row=ROW+1, column=2, sticky=W, padx=5, pady=(0,20))
        self.TemplateTip = ('The template is a starting point for\nthe ' +
                            'application.  It will provide the\nbasic ' +
                            'framework from which custom\napplication ' +
                            'development can begin.')
        createToolTip(self.Template, self.TemplateTip, 3)

        # Generate Eclipse Project
        self.EclipseVal = IntVar()
        self.Eclipse = Checkbutton(frame, text="Generate Eclipse Project",
                                   variable=self.EclipseVal, state=NORMAL)
        self.Eclipse.grid(row=ROW, rowspan=2, column=2, sticky=E, padx=5)
        self.EclipseVal.set(True)
        self.EclipseTip = ('Generate an Eclipse IDE project\n'
                           'for the application.')
        createToolTip(self.Eclipse, self.EclipseTip, 20)

        ROW += 2

        # Targets
        self.TargetLabel = Label(frame, text="Targets", font="Arial 12 bold")
        self.TargetLabel.grid(row=ROW, column=0, sticky=S+W, padx=10)

        self.LinuxX86 = ['linux.x86', IntVar(), None]
        self.add_target(self.LinuxX86, frame, ROW+1, None)

        self.MachX86_64 = ['mach.x86_64', IntVar(), None]
        self.add_target(self.MachX86_64, frame, ROW+2, None)

        self.FreertosArmv7m = ['freertos.armv7m (Cortex-M3/M4)', IntVar(), None]
        self.add_subtargets(self.FreertosArmv7m, frame, ROW+3, FreertosArmv7mList)

        ROW += 11

        # Quit Button
        self.Exit = Button(frame, text="Quit", command=frame.quit)
        self.Exit.grid(row=ROW-2, column=2, rowspan=3, columnspan=2, sticky=S+E, padx=5, pady=5)

        # Generate Button
        self.Generate = Button(frame, text="Generate", command=frame.quit)
        self.Generate.grid(row=ROW-2, column=2, rowspan=3, columnspan=2, sticky=S+E, padx=(0,95), pady=5)

    # create a path entry box
    def path_entry(self, frame, item, label, value, tip, row, column, browse=True):
        label = Label(frame, text=label, font="Arial 12 bold")
        label.grid(row=row, column=column, sticky=S+W, padx=10)
        item = Entry(frame, width=50)
        item.grid(row=row+1, column=column, sticky=E+W, padx=5, pady=(0,20))
        if value != None:
            item.insert(0, value)
        if tip != None:
            createToolTip(item, tip, 3)
        if browse == True:
            button = Button(frame, text='Browse...', command=partial(self.browse, item))
            button.grid(row=row+1, column=column+1, sticky=W, padx=5, pady=(0,20))

    # create a target
    def add_target(self, target, frame, row, target_list):
        target[2] = Checkbutton(frame, text=target[0],
                                variable=target[1], state=NORMAL,
                                command=partial(self.target_select, target, target_list))
        target[2].grid(row=row, column=0, sticky=W, padx=5)

    # create a sub-target list
    def add_subtargets(self, target, frame, row, target_list):
        self.add_target(target, frame, row, target_list)
        row += 1
        for item in target_list:
            item[1] = IntVar()
            item[2] = Checkbutton(frame, text=item[0],
                                  variable=item[1], state=DISABLED)
            item[2].grid(row=row, column=0, sticky=W, padx=25)
            row += 1

    def target_select(self, target, target_list):
        if target_list == None:
            return
        if target[1].get():
            for item in target_list:
                item[2].configure(state=NORMAL)
        else:
            for item in target_list:
                item[2].configure(state=DISABLED)
                item[1].set(False)

    def browse(self, box):
        box.insert(0, askdirectory(**self.dir_opt))

# Start of the application
root = Tk()

app = App(root)

root.mainloop()
