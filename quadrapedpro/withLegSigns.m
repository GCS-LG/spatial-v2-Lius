function Location = withLegSigns(L, legID)

if(legID == 0)
    Location = [ L(1), -L(2), L(3) ];
elseif(legID == 1)
    Location = [ L(1),  L(2), L(3) ];
elseif(legID == 2)
    Location = [ -L(1), -L(2), L(3) ];
elseif(legID == 3)
    Location = [ -L(1), L(2), L(3) ];
end

end