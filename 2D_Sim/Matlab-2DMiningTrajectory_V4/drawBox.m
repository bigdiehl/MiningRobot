function handle = drawBox(z, L, handle)

    pts = [ z+0, z+L, z+L, z+0; 0, 0, L, L]; %Add z value to box
    %of size L with left edge centered at z = 0. Translates box
    %left and right along z-axis.
    
    X = pts(1,:);
    Y = pts(2,:);

    if isempty(handle)
        handle = fill(X,Y,'b');
    else
        set(handle,'XData',X,'YData',Y);
    end
end

