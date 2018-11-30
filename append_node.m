function tree = append_node(tree,new_node,parent_pointer)
node.p = new_node;
node.parent_pointer = parent_pointer;
tree{end+1} = node;
end