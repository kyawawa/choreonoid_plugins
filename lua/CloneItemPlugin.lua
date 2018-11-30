cnoid.require "Util"
cnoid.require "Base"
cnoid.require "Body"
cnoid.require "BodyPlugin"

half_side_num = 20 -- (var * 2) * (var * 2) - 1
interval = 0.05

function deepCopy (t) -- deep-copy a table
    if type(t) ~= "table" then return t end
    local meta = getmetatable(t)
    local target = {}
    for k, v in pairs(t) do
        if type(v) == "table" then
            target[k] = deepCopy(v)
        else
            target[k] = v
        end
    end
    setmetatable(target, meta)
    return target
end

button = toolBar:addButton("Clone")
signal = button:sigToggled()
connection = signal:connect(cloneItem)

function cloneItem()
   item = cnoid.ItemTreeView.instance():selectedItems()[0]
   parent = item:parentItem()

   for i = 1, (half_side_num * 2 - 1) do
      for j = 1, (half_side_num * 2 - 1) do
         if i ~= half_side_num and j ~= half_side_num then
            duplicated = deepCopy(item)
            root = duplicated:body():rootLink()
            pos = root:position()

            pos[0][3] = (i - half_side_num) * interval -- x
            pos[1][3] = (j - half_side_num) * interval  -- y
            root.setPosition(pos)
            parent:addChildItem(duplicated)
         end
      end
   end

end
