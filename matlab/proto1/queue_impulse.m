% Queue up impulse to be applied when integrate_vel is called
function body = queue_impulse( body, impulse, loc )
    body.impulses = [body.impulses; impulse loc];
end