class Node:
    
    def  __init__(self, left = None, right = None, isconcave = 1, area = None, id = 0, height = 0):
        self.left = left
        self.right = right
        self.isconcave = isconcave
        self.area = area
        self.id = id
        self.height = height
    

class Tree:
    
    def __init__(self, root):
        self.root = root
    
    def traversal(self):
        stack = []
        hash = {}
        hash_total = {}
        node = self.root
        while stack or node:
            while node:
                stack.append(node)
                node = node.left
            node = stack.pop()
            hash_total[node.id] = node
            if node.isconcave == 1 and node.left == None and node.right == None:
                hash[node.id] = node
            else:

                pass
            node = node.right
        for id, node in hash.items():
            print("id: {}, val: {}, is_concave: {} height: {}".format(id, node.area, node.isconcave, node.height))
            
        print("")

        for id, node in hash.items():
            if node.isconcave == 1:
                node.isconcave = "passed"
        
        for id, node in hash.items():
            print("id: {}, val: {}, is_concave: {} height: {}".format(id, node.area, node.isconcave, node.height))
            
        print("")
        
        for id, node in hash_total.items():
            print("??")
            print("id: {}, val: {}, is_concave: {} height: {}".format(id, node.area, node.isconcave, node.height))

        

                
    
    def grow(self, root, a, b, a_isconcave, b_isconcave):
        node_left = Node(None, None, a_isconcave, a)
        print(node_left.id)
        node_left.id = root.id * 10 + 1
        node_left.height = root.height + 1
        node_right = Node(None, None, b_isconcave, b)
        node_right.id = root.id * 10 + 2
        node_right.height = node_left.height
        root.left = node_left
        root.right = node_right



        
        
testTree = Tree(Node(None, None, 1, "test root", 0))
print("a")
print(Node(None, None, 1, "test root", 0).area)
testTree.grow(testTree.root, "a", "b", 1, 0)
testTree.grow(testTree.root.left, "c", "d", 0, 1)
testTree.grow(testTree.root.left.right, "e", "f", 1, 1)
testTree.traversal()



