function cdata = zbuffer_cdata(hfig)
% Get CDATA from hardcopy using zbuffer
% http://www.mathworks.com/support/solutions/en/data/1-3NMHJ5/?solution=1-3NMHJ5

% Need to have PaperPositionMode be auto
orig_mode = get(hfig, 'PaperPositionMode');
set(hfig, 'PaperPositionMode', 'auto');

cdata = hardcopy(hfig, '-Dzbuffer', '-r0');
cdata = cdata(58:523,78:543,:);
% Restore figure to original state
set(hfig, 'PaperPositionMode', orig_mode); % end
end
