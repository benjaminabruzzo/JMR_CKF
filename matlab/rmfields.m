% clip = rmfields(clip, {'timeOfRefVector', 'referenceVector'})
function s = rmfields(s, cells_of_fields)
  try
    for idx = 1:length(cells_of_fields)
      s = rmfield(s, cells_of_fields{idx});
    end
  catch Mexc_; disp([Mexc_.identifier ' :: ' Mexc_.message]); end
end