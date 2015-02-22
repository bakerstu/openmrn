#!/usr/bin/python

import os
import sys
import serial
import glob
from Tkinter import *
from ttk import *
import Image

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
        ROW = 0

        #frame.columnconfigure(1, weight=1)
        #frame.columnconfigure(2, weight=3)
        #frame.columnconfigure(3, weight=1)
        #frame.rowconfigure(1, weight=10)
        #frame.rowconfigure(2, weight=1)

        # OpenMRN Path
        self.OpenmrnLabel = Label(frame, text="OpenMRN Path",
                                  font="Arial 12 bold")
        self.OpenmrnLabel.grid(row=ROW, column=0, columnspan=2, sticky=S+W, padx=10)
        self.OpenmrnTip = ('This is the file system path which points to\nthe '
                           'location of the OpenMRN project tree.')
        self.OpenmrnPath = Entry(frame, width=64)
        self.OpenmrnPath.grid(row=ROW+1, column=0, columnspan=2, sticky=E+W, padx=5, pady=(0,20))
        self.OpenmrnPath.insert(0, os.path.abspath(os.getcwd() + '/../'))
        createToolTip(self.OpenmrnPath, self.OpenmrnTip, 3)

        # Application Path
        self.ApplLabel = Label(frame, text="New Application Path",
                               font="Arial 12 bold")
        self.ApplLabel.grid(row=ROW, column=2, columnspan=2, sticky=S+W, padx=10)
        self.ApplPath = Entry(frame, width=64)
        self.ApplPath.grid(row=ROW+1, column=2, columnspan=2, sticky=E+W, padx=5, pady=(0,20))
        self.ApplTip = ('This is the file system path which points to\nthe '
                        'location of the generated application.')
        createToolTip(self.ApplPath, self.ApplTip, 3)

        ROW +=2

        # Application Sub-directories
        self.SubdirLabel = Label(frame, text="Application Subdirectory List [space separated]",
                                 font="Arial 12 bold")
        self.SubdirLabel.grid(row=ROW, column=0, columnspan=2, sticky=S+W, padx=10)
        self.SubdirEntry = Entry(frame, width=64)
        self.SubdirEntry.grid(row=ROW+1, column=0, columnspan=2, sticky=E+W, padx=5, pady=(0,20))
        self.SubdirTip = ('Often it is usefull to add sub-directories\nto '
                          'better organize a large project.  The\nprovided '
                          'list of sub-directories will be\nadded to the '
                          'build rules for the\napplication.  Additional sub-'
                          'directories\ncan be added later by using the\n'
                          '"SubdirAdder" program.')
        createToolTip(self.SubdirEntry, self.SubdirTip, 3)

        # Template
        self.TemplateLabel = Label(frame, text="Template",
                                   font="Arial 12 bold")
        self.TemplateLabel.grid(row=ROW, column=2, sticky=S+W, padx=10)
        self.Template = Combobox(frame, textvariable=StringVar(), state="readonly")
        #self.Template.bind('<<ComboboxSelected>>', self.template_select)
        self.Template['values'] = ('empty', 'blink', 'train', 'traction proxy', 'hub')
        self.Template.current(0)
        self.Template.grid(row=ROW+1, column=2, sticky=W+E, padx=5, pady=(0,20))
        self.TemplateTip = ('The template is a starting point for\nthe ' +
                            'application.  It will provide the\nbasic ' +
                            'framework from which custom\napplication ' +
                            'development can begin.')
        createToolTip(self.Template, self.TemplateTip, 3)

        # Generate Eclipse Project
        self.EclipseVal = IntVar()
        self.Eclipse = Checkbutton(frame, text="Generate Eclipse Project",
                                   variable=self.EclipseVal, state=NORMAL)
        self.Eclipse.grid(row=ROW, rowspan=2, column=3, sticky=W, padx=5)
        self.EclipseVal.set(True)
        self.EclipseTip = ('Generate an Eclipse IDE project\n'
                           'for the application.')
        createToolTip(self.Eclipse, self.EclipseTip, 20)

        ROW += 2

        # Targets
        self.TargetLabel = Label(frame, text="Targets", font="Arial 12 bold")
        self.TargetLabel.grid(row=ROW, column=0, sticky=S+W, padx=10)
        self.LinuxX86Val = IntVar()
        self.LinuxX86 = Checkbutton(frame, text="linux.x86",
                                    variable=self.LinuxX86Val,
                                    state=NORMAL)
        self.LinuxX86.grid(row=ROW, column=0, sticky=W, padx=5)
        ROW += 1
        self.MachX86_64Val = IntVar()
        self.MachX86_64 = Checkbutton(frame, text="mach.x86_64",
                                       variable=self.MachX86_64Val,
                                       state=NORMAL)
        self.MachX86_64.grid(row=ROW, column=0, sticky=W, padx=5)
        ROW += 1
        self.FreertosArmv7mVal = IntVar()
        self.FreertosArmv7m = Checkbutton(frame, text="freertos.armv7m",
                                          variable=self.FreertosArmv7mVal,
                                          state=['!selected'])
        self.FreertosArmv7m.grid(row=ROW, column=0, sticky=W, padx=5)
        ROW += 1
        self.Eklm4f120xlVal = IntVar()
        self.Eklm4f120xl = Checkbutton(frame, text="ek-lm4f120xl (legacy)",
                                       variable=self.Eklm4f120xlVal,
                                       state=DISABLED)
        self.Eklm4f120xl.grid(row=ROW+1, column=0, sticky=W, padx=25)
        createToolImage(self.Eklm4f120xl, "images/ek-tm4c123gxl.gif")
        self.Ektm4c123gxlVal = IntVar()
        self.Ektm4c123gxl = Checkbutton(frame, text="ek-tm4c123gxl",
                                        variable=self.Ektm4c123gxlVal,
                                        state=DISABLED)
        self.Ektm4c123gxl.grid(row=ROW+2, column=0, sticky=W, padx=25)
        createToolImage(self.Ektm4c123gxl, "images/ek-tm4c123gxl.gif")
        self.Ektm4c1294xlVal = IntVar()
        self.Ektm4c1294xl = Checkbutton(frame, text="ek-tm4c1294xl",
                                        variable=self.Ektm4c1294xlVal,
                                        state=DISABLED)
        self.Ektm4c1294xl.grid(row=ROW+3, column=0, sticky=W, padx=25)
        createToolImage(self.Ektm4c1294xl, "images/ek-tm4c1294xl.gif")

        ROW += 4

        # Quit Button
        self.Exit = Button(frame, text="Quit", command=frame.quit)
        #self.Exit.pack(side=RIGHT)
        self.Exit.grid(row=ROW-2, column=3, rowspan=3, sticky=S+E, padx=5, pady=5)

        # Generate Button
        self.Generate = Button(frame, text="Generate", command=frame.quit)
        #self.Generate.pack(side=RIGHT)
        self.Generate.grid(row=ROW-2, column=3, rowspan=3, sticky=S, padx=(0,15), pady=5)

# Start of the application
root = Tk()

app = App(root)

root.mainloop()
