#include <iostream>
#include <utility>
#include <memory>

#include <opencsg/opencsg.h>
// For compilers that support precompilation, includes "wx/wx.h".
#include <wx/wxprec.h>
#ifndef WX_PRECOMP
    #include <wx/wx.h>
#endif

#include <wx/glcanvas.h>

enum
{
    ID_Hello = 1
};


class MyFrame: public wxFrame
{
    wxGLCanvas *m_canvas = nullptr;
    wxGLContext *m_context = nullptr;
public:
    MyFrame(const wxString& title, const wxPoint& pos, const wxSize& size): 
        wxFrame(nullptr, wxID_ANY, title, pos, size)
    {
        wxMenu *menuFile = new wxMenu;
        menuFile->Append(ID_Hello, "&Hello...\tCtrl-H",
                         "Help string shown in status bar for this menu item");
        menuFile->AppendSeparator();
        menuFile->Append(wxID_EXIT);
        wxMenu *menuHelp = new wxMenu;
        menuHelp->Append(wxID_ABOUT);
        wxMenuBar *menuBar = new wxMenuBar;
        menuBar->Append( menuFile, "&File" );
        menuBar->Append( menuHelp, "&Help" );
        SetMenuBar( menuBar );
        CreateStatusBar();
        SetStatusText( "Welcome to wxWidgets!" );
        
        m_canvas = new wxGLCanvas(this, {});
        m_context = new wxGLContext(m_canvas);
        
        Bind(wxEVT_MENU, &MyFrame::OnHello, this, ID_Hello);
        Bind(wxEVT_MENU, &MyFrame::OnExit, this, wxID_EXIT);
        Bind(wxEVT_MENU, &MyFrame::OnAbout, this, wxID_ABOUT);
    }
private:
    void OnHello(wxCommandEvent& /*event*/) 
    {
        std::cout << "Hello Man" << std::endl;
    }
    
    void OnExit(wxCommandEvent& /*event*/) 
    {
        Close( true );
    }
    
    void OnAbout(wxCommandEvent& /*event*/) 
    {
        wxMessageBox( "This is a wxWidgets' Hello world sample",
                      "About Hello World", wxOK | wxICON_INFORMATION );
    }
};

class App : public wxApp {
    MyFrame *m_frame;
public:
    bool OnInit() override {
        m_frame = new MyFrame( "Hello World", wxPoint(50, 50), wxSize(450, 340) );
        m_frame->Show( true );
        
        return true;
    }    
};

wxIMPLEMENT_APP(App);
