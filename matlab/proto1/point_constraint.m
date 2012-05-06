function [xtarget, qtarget] = point_constraint( xnew, qnew )
    xtarget = [0 0 0];
    qtarget = qnew;
end