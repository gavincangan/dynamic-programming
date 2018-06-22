clear;

%Number of nodes
n = 25;

%Number of robots
robots = 3;

%Budget
budget = 5;

% Set value between -0.5 and 0.5
% Controls formations of edges
% Higher value --> more edges formed
control_edges = -0.37;

remove_two_node_loops = true;

color_array = ['y', 'm', 'c', 'r', 'g', 'b', 'k'];

assert(robots<=size(color_array,2) && robots <= n/3,'Too many robots!');

AdjM = round(rand(n) + control_edges );

%Add only for undirected graphs
%AdjM = triu(AdjM) + triu(AdjM,1)';

AdjM = AdjM - diag(diag(AdjM));

% Remove two-node loops
% For now, arbirarily removes the arrow
% pointing to the lower index node
% Need to make this more random :-)
if(remove_two_node_loops == true)
    AdjM = AdjM & (~triu(AdjM))';
end

%Weight vector - randomly assigned weights
W_vect = round( 25 * rand(n,1) );

%Score vector - calculated score for each node
S_vect = zeros(n,1);

G = digraph(AdjM);

E = G.Edges;
N = G.Nodes;

G.Nodes.Weight = W_vect;

%Sort both ways (rows and columns) based on the number of outgoing edges
G_mat = [AdjM zeros(n,2)];
G_mat(1:n,n+1) = sum(G_mat(1:n,1:n),2);
G_mat(1:n,n+2) = 1:n;
G_mat = sortrows(G_mat,n+1);

G_mat = [G_mat; zeros(2, n+2)];
G_mat(n+1,1:n) = sum(G_mat(1:n,1:n),1);
G_mat = transpose(G_mat);
G_mat(1:n,n+2) = 1:n;
G_mat(1:n,:) = sortrows(G_mat(1:n,:),n+1);
G_mat = transpose(G_mat);

for colId = 1:n
    temp_vect = AdjM(:,colId);
    temp_vect(colId) = 0;
    temp_vect = temp_vect  .* W_vect;
    S_vect = S_vect + temp_vect;
end

S_vect = S_vect + W_vect;

S_mat = [S_vect (1:n)'];

G.Nodes.Score = S_vect;

G_nodeLabels = cell(n,1);

for i= 1:n
    temp = sprintf('#%2d-W%2d|S%2d',i, W_vect(i) ,S_vect(i));
    G_nodeLabels{i} = (temp);
end

G.Nodes.Name = G_nodeLabels;

S_mat_sort = sortrows(S_mat,1,'descend');

for i = 1:n
    fprintf("%d:\t%d, %d\n",i,S_vect(i),W_vect(i));
end

g_handle = plot(G,'NodeLabel',G.Nodes.Name);

highlight(g_handle, S_mat_sort(1:robots,2)');

% To verify that all the sort and transpose operations
% have not messed up the data in the array :-)
%{
Ver_G_mat = G_mat;
Ver_G_mat(n+1:n+2, n+1:n+2) = [1000, 1001; 1002, 1003];

Ver_G_mat(1:n,1:n+2) = sortrows(Ver_G_mat(1:n,1:n+2),n+2);
Ver_G_mat = transpose(Ver_G_mat);
Ver_G_mat(1:n,1:n+2) = sortrows(Ver_G_mat(1:n,1:n+2),n+2);
Ver_G_mat = transpose(Ver_G_mat);
%}

score_array = transpose( S_mat );

pathMat = zeros(robots,budget+1);

pathMat(:,1) = S_mat_sort(1:robots,2);      % Copy initial positions determined by getting max scores
for r = 1:robots
    score_array(1, ( pathMat(r,1) ) ) = 0;  % Set the score of initial position nodes to zero
end

end_of_path = false(robots,1);

%Begin robot motion - Greedy
for b = 2:budget+1
    for r = 1:robots
        if(~end_of_path(r))
            poss_moves = AdjM( pathMat(r,b-1), : ) .* score_array(1,:) ;
            [max_val, max_indx] = max( poss_moves );
            if(max_val ~= 0)
                pathMat(r,b) = max_indx;
            else
                end_of_path(r,1) = true;
            end
            score_array(1, max_indx) = 0;
        end
    end
end

%Highlight nodes on the graph
for r = 1:robots
   temp_array = pathMat(r,:);
   
   temp_array( ~any(temp_array,2), : ) = [];  %rows
   temp_array( :, ~any(temp_array,1) ) = [];  %columns

   highlight( g_handle, temp_array ,'NodeColor', color_array(r) );
end


%Highlight edges on the graph #Attempt2
%Add all nodes first and then add edges corresponding to robots path
%{
for r = 1:robots
    Q = digraph();
    Q = addnode(Q,10);

    for b = 1:budget
       if( pathMat(r,b+1) ~=0 ) 
            Q = addedge( Q, pathMat(r,b), pathMat(r,b+1) );
       end
    end
    Q
    figure; plot(Q);
    highlight(g_handle, Q ,'EdgeColor',color_array(r), 'LineWidth',1.5);
end
%}


%Highlight edges on the graph #Attempt1
%Add edges corresponding to robots path and then add remaining nodes
%{
for r = 1:robots
    
   %nodes_to_add = ones(1,n);
   
   s_array = zeros(1,budget);
   t_array = zeros(1,budget);

   s_array(1,1) = pathMat(r,1);
   t_array(1,1) = pathMat(r,2);
   
   %t_array(1,pathMat(r,1)) = pathMat(r,2);
   
   for b = 2:budget
       
       %t_array( 1, pathMat(r,b) ) = pathMat(r,b+1);

       s_array(r,b) = pathMat(r,b);
       t_array(r,b) = pathMat(r,b+1);
       
    %Assign zero to s and t if either is zero
       if( (s_array(r,b) == 0) || (t_array(r,b) == 0) )
           t_array(r,b) = 0;
           s_array(r,b) = 0;
       end
       
   end

   %Remove zeros because they do not make sense to Graph constructor
   s_array( ~any(s_array,2), : ) = [];  %rows
   s_array( :, ~any(s_array,1) ) = [];  %columns
   t_array( ~any(t_array,2), : ) = [];  %rows
   t_array( :, ~any(t_array,1) ) = [];  %columns
   
   %s_array
   %t_array
   
   temp_graph = G;
   
%    for index = 1:size(s_array,2)
%        nodes_to_add( s_array(1,index) ) = 0;
%        nodes_to_add( t_array(1,index) ) = 0;
%    end
  
  %numnodes(temp_graph)
  %temp_graph
  for index = numnodes(temp_graph):n
      temp_graph.addnode(index)
      %disp('inc');
  end
  
    %numnodes(temp_graph)
    %temp_graph
  for index = numnodes(temp_graph):n
      temp_graph.rmnode(index);
      %disp('dec');
  end
   
   %figure; plot(temp_graph);
   
   highlight(g_handle, temp_graph ,'EdgeColor',color_array(r), 'LineWidth',1.5);
   %highlight(h,T,'EdgeColor','r','LineWidth',1.5)
end

%}

