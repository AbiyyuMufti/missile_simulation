function simpleGUI
    hFig = figure('Visible','off', 'Menu','none', 'Name','Calculator', 'Resize','off', 'Position',[100 100 750 400]);
    movegui(hFig,'center')          %# Move the GUI to the center of the screen

    hBtnGrp = uibuttongroup('Position',[0 0 0.3 1], 'Units','Normalized');

%Working buttons
    uicontrol('Style','pushbutton', 'Parent',hBtnGrp, 'String','Compute', 'Position',[15 265 70 30], 'Callback',{@button_callback})
    uicontrol('Style','text', 'Parent',hBtnGrp, 'HandleVisibility','off', 'Position',[15 345 70 30], 'String','(w*d)/(2*U)')
    uicontrol('Style','text', 'Parent',hBtnGrp, 'HandleVisibility','off', 'Position',[15 305 70 30], 'String','C_l')
       
    hEdit1 = uicontrol('Style','edit', 'Parent',hBtnGrp, 'Position',[20 338 60 20], 'String','3');
    hEdit3 = uicontrol('Style','edit', 'Parent',hBtnGrp, 'Position',[20  298 60 20], 'String','');

%Disclaimer for Reynolds number
    uicontrol('Style','text', 'Parent',hBtnGrp, 'HandleVisibility','off', 'Position',[15 185 125 50], 'String','Relevent for Reynolds number of 6e4')

%Radio buttons
    uicontrol('Style','text', 'Parent',hBtnGrp, 'HandleVisibility','off', 'Position',[15 150 70 30], 'String','Roughness')
 
    uicontrol('Style','Radio', 'Parent',hBtnGrp, 'HandleVisibility','off', 'Position',[15 130 70 30], 'String','Smooth', 'Tag','+')
    uicontrol('Style','Radio', 'Parent',hBtnGrp, 'HandleVisibility','off', 'Position',[15 100 70 30], 'String','5E-6 m', 'Tag','-')
    uicontrol('Style','Radio', 'Parent',hBtnGrp, 'HandleVisibility','off', 'Position',[15  70 70 30], 'String','1E-5 m', 'Tag','*')
    uicontrol('Style','Radio', 'Parent',hBtnGrp, 'HandleVisibility','off', 'Position',[15  40 70 30], 'String','5E-5 m', 'Tag','/')
    uicontrol('Style','Radio', 'Parent',hBtnGrp, 'HandleVisibility','off', 'Position',[15  10 70 30], 'String','1E-4 m', 'Tag','%')
  
    set(hFig, 'Visible','on')        %# Make the GUI visible

    %# callback function
    function button_callback(src,ev)
        x = str2double(get(hEdit1, 'String'));
        if x<1 || x>4
                errordlg('Input must be a number greater than 1 or less than 4','Error');
        else
          y=1:.01:4;
          switch get(get(hBtnGrp,'SelectedObject'),'Tag')
            case '+',  res =.0049025*x.^3-.040825*x.^2+.096596*x+.29093;
                plot(1:.01:4,.0049025*y.^3-.040825*y.^2+.096596*y+.29093,x,res,'*');
                title('c_l vs (w*d)/(2U)')
                set(gca,'yaxislocation','right');
                xlabel('c_l');
                ylabel('(w*d)/(2U)');
                legend('y=.0049025x^3-.040825x^2+.096596x+.29093');
            case '-',  res =.0049448*x.^3-.041141*x.^2+.097281*x+.29054;
                plot(1:.01:4,.0049448*y.^3-.041141*y.^2+.097281*y+.29054,x,res,'*');
                title('c_l vs (w*d)/(2U)')
                set(gca,'yaxislocation','right');
                xlabel('c_l');
                ylabel('(w*d)/(2U)');
                legend('y=.0049448x^3-.041141x^2+.097281x+.29054');
            case '*',  res =.0049856*x.^3-.041446*x.^2+.097943*x+.29017;
                plot(1:.01:4,.0049856*y.^3-.041446*y.^2+.097943*y+.29017,x,res,'*');
                title('c_l vs (w*d)/(2U)')
                set(gca,'yaxislocation','right');
                xlabel('c_l');
                ylabel('(w*d)/(2U)');
                legend('y=.0049856x^3-.041446x^2+.097943x+.29017');
            case '/',  res =.0052632*x.^3-.043517*x.^2+.10241*x+.28768;
                plot(1:.01:4,.0052632*y.^3-.043517*y.^2+.10241*y+.28768,x,res,'*');
                title('c_l vs (w*d)/(2U)')
                set(gca,'yaxislocation','right');
                xlabel('c_l');
                ylabel('(w*d)/(2U)');
                legend('y=.0052632x^3-.043517x^2+.10241x+.28768');
            case '%',  res =.0056115*x.^3-.046072*x.^2+.10789*x+.28462;
                plot(1:.01:4,.0056115*y.^3-.046072*y.^2+.10789*y+.28462,x,res,'*');
                title('c_l vs (w*d)/(2U)')
                set(gca,'yaxislocation','right');
                xlabel('c_l');
                ylabel('(w*d)/(2U)');
                legend('y=.0056115x^3-.046072x^2+.10789x+.28462');
            otherwise, res = '';
         end
        set(hEdit3, 'String',res)
       end
    end
end

%------------------------------------------------------------------------
%Data taken from Star CCM*
%smooth=.0049025*x.^3-.040825*x.^2+.096596*x+.29093
%5E-6=.0049448*x.^3-.041141*x.^2+.097281*x+.29054;
%1E-5=.0049856*x.^3-.041446*x.^2+.097943*x+.29017;
%5E-5=.0052632*x.^3-.043517*x.^2+.10241*x+.28768;
%5E-5=.0056115*x.^3-.046072*x.^2+.10789*x+.28462;