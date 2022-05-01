function mojave = get_mojave()
%GET_MOJAVE Runs proper setup of the mojave and returns it
    try
        fclose(instrfindall);
        delete(instrfindall);
    catch
    end
    evalin('base', 'clear');
    input("Turn off the Mojave then hit enter.");
    mojave = robot;
    mojave.set_speed(-1);
    input("You may now turn on the Mojave.");
end

