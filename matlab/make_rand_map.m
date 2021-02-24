  s = rng('shuffle');
  center = [1 0];
  spread = [3.5 2.5];

  gen_rand_xy = @(x,y) (rand(x,y)-[0.5 0.5]).* spread + center;
  gen_rand_yaw = @(x,y) 2*pi.*rand(x,y);

  tags = {...
    'id08', '08';...
    'id01', '01';...
    'id02', '02';...
    'id03', '03';...
    'id04', '04';...
    'id05', '05';...
    'id06', '06';...
    'id07', '07';...
    'id09', '09';...
    'id10', '10';...
    'id11', '11';...
    };

  X = gen_rand_xy(1,2);

  while (size(X,1) < size(tags,1))
    x = gen_rand_xy(1,2);
    for kdx = 1:size(X,1)
      x_diff(kdx,1) = sqrt((X(kdx,:)-x)*(X(kdx,:)-x)');
    end; clear kdx

    if ~(any(x_diff<.45))
      X = [X;x];
%       disp(size(X))
    else
      % do nothing / repeat loop
    end

  end; clear idx

  X =[X, gen_rand_yaw(size(X,1),1)];

    % figure(1); clf;
    % hold on
    %   try plot(X(:,1), X(:,2), 'k*'); catch; end
    %   axis([-3 8 -4 4])
    % hold off
    % grid on


%% Write to file
  Launch.filename = ['/home/benjamin/ros/src/metahast/robot_descriptions/tags/launch/tags_rand.launch'];
  Launch.fileID = fopen(Launch.filename,'w');
  fprintf(Launch.fileID, '<!-- -*- mode: XML -*- -->\n\n');
  fprintf(Launch.fileID, '<launch>\n\n');
  for idx = 1:size(tags,1)
    fprintf(Launch.fileID, '\t<include file="$(find robot_descriptions)/tags/launch/spawn_tag.launch" >\n');
    fprintf(Launch.fileID, '\t\t<arg name="tag"\tvalue="%s"/>\n', tags{idx,1});
    fprintf(Launch.fileID, '\t\t<arg name="id"\tvalue="%s"/>\n', tags{idx,2});
    fprintf(Launch.fileID, '\t\t<arg name="x"\t\tvalue="%6.3f"/>\n', X(idx,1));
    fprintf(Launch.fileID, '\t\t<arg name="y"\t\tvalue="%6.3f"/>\n', X(idx,2));
    fprintf(Launch.fileID, '\t\t<arg name="Y"\t\tvalue="%6.3f"/>\n', X(idx,3));
    fprintf(Launch.fileID, '\t</include>\n\n');
  end; clear idx

  fprintf(Launch.fileID, '</launch>\n');
