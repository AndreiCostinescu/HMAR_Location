function [ out ] = RootDirRemoval( in )
%DIRCHECK Summary of this function goes here
%   Detailed explanation goes here

if (strcmp(in,'.') || strcmp(in,'..'))
    out = true;
else
    out = false;
end

end
