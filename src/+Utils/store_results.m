function status = store_results(TOD,frameCount,StopTime, Frames, cav_env,...
    tf, i_m, comment)
% store_results store results from lane changing maneuvers while creating
% plots

    % Create containing folder 
    location = strcat('.',filesep,'Results',filesep,TOD); 
    status = mkdir(location);
    filename = strcat(location,filesep,'test_',TOD);
    fprintf("Generating video...\n")
    writerObj = VideoWriter(strcat(filename,'.mp4'),'MPEG-4');
    writerObj.FrameRate = round(frameCount/StopTime);
    open(writerObj)
    writeVideo(writerObj, Frames);
    close(writerObj);
    % Store variables
    save(strcat(filename,'.mat'), "cav_env","tf","i_m",'-mat')
    % Create plots and save them
    Utils.plot_state_history(cav_env, tf, i_m, location)
    % Provide a readme file containing experiments details
    fid = fopen(strcat(location,filesep,'README.txt'), 'w');
    % Write the time of day to the file
    fprintf(fid, 'Time of day: %s\n', TOD);
    % Write the simulation length to the file
    fprintf(fid, 'Simulation length: %d seconds\n', StopTime);
    % Write comments to the file
    fprintf(fid, 'Comments:\n');
    fprintf(fid, '%s \n', comment);
    fclose(fid);
end

